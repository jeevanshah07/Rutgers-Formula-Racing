const std = @import("std");

pub const FrameKind = enum { standard, extended };
pub const Endian = enum { little, big };

pub const Frame = struct {
    id: u32,
    kind: FrameKind,
    remote: bool = false,
    dlc: u4,
    data: [8]u8,
};

pub const VoltageSpec = struct {
    id: u32,
    kind: FrameKind,
    offset: u3, // bytes from the start
    width: enum { one, two, four }, // in bytes
    endian: Endian,
    multiplier: u32 = 1,
    divisor: u32 = 1,
    cartOffset: ?u3 = null,
};

pub const ProtocolConfig = struct {
    ready: bool,
    bitrate: u32,
    bms: VoltageSpec,
    inverter: VoltageSpec,
    min_voltage: u32 = 0,
    max_voltage: u32 = 450,
    threshold_percent: u8 = 90,
    qualifying_samples: u8 = 3,
    freshness_timeout_ms: u32 = 10_000, // 10 sec
    precharge_timeout_ms: u32 = 300_000, // 5 min
};

pub const base_config = ProtocolConfig{
    .ready = true,
    .bitrate = 500_000,
    .bms = .{ .id = 0x6B1, .kind = .standard, .offset = 2, .width = .two, .endian = .big, .multiplier = 1, .cartOffset = 5 },
    .inverter = .{ .id = 0x0A7, .kind = .extended, .offset = 0, .width = .two, .endian = .little, .divisor = 10 },
};

pub const DecodeError = error{
    RemoteFrame,
    WrongFrame,
    PayloadTooShort,
    InvalidScale,
    InvalidCartFlag,
    Overflow,
};

pub fn checkCart(frame: Frame, spec: VoltageSpec) DecodeError!bool {
    if (frame.remote) return error.RemoteFrame;
    if (frame.id != spec.id or frame.kind != spec.kind) return error.WrongFrame;

    const start = spec.cartOffset orelse return false;
    if (@as(usize, frame.dlc) <= @as(usize, start)) return error.PayloadTooShort;

    const raw: u32 = frame.data[start];
    switch (raw) {
        0 => return true, // 0 indicates charging is happening, which can only happen on the cart
        1 => return false,
        else => return error.InvalidCartFlag,
    }
}

pub fn decodeVoltage(frame: Frame, spec: VoltageSpec) DecodeError!u32 {
    if (frame.remote) return error.RemoteFrame;
    if (frame.id != spec.id or frame.kind != spec.kind) return error.WrongFrame;

    const width: usize = switch (spec.width) {
        .one => 1,
        .two => 2,
        .four => 4,
    };
    const start: usize = spec.offset;
    if (@as(usize, frame.dlc) < start + width) return error.PayloadTooShort;
    if (spec.divisor == 0) return error.InvalidScale;

    const raw: u32 = switch (spec.width) {
        .one => frame.data[start],
        .two => switch (spec.endian) {
            .little => std.mem.readInt(u16, frame.data[start..][0..2], .little),
            .big => std.mem.readInt(u16, frame.data[start..][0..2], .big),
        },
        .four => switch (spec.endian) {
            .little => std.mem.readInt(u32, frame.data[start..][0..4], .little),
            .big => std.mem.readInt(u32, frame.data[start..][0..4], .big),
        },
    };
    const scaled = @mulWithOverflow(raw, spec.multiplier);
    if (scaled[1] != 0) return error.Overflow;
    return scaled[0] / spec.divisor;
}

pub const State = enum { waiting_for_bms, precharging, complete, fault_latched };
pub const Fault = enum(u8) {
    stale_bms = 1,
    stale_inverter = 2,
    configuration = 3,
    malformed_frame = 4,
    implausible_voltage = 5,
    timeout = 6,
};

pub const info_can_id: u32 = 0x0D3;

pub fn infoFrame(controller: Controller) Frame {
    var frame = Frame{ .id = info_can_id, .kind = .standard, .dlc = 5, .data = @splat(0) };
    frame.data[0] = if (controller.fault) |fault| @intFromEnum(fault) else switch (controller.state) {
        .complete => 7,
        .precharging => 8,
        else => 0
    };
    if (controller.bms) |reading|
        std.mem.writeInt(u16, frame.data[1..3], @intCast(reading.value), .big);
    if (controller.inverter) |reading|
        std.mem.writeInt(u16, frame.data[3..5], @intCast(reading.value), .big);
    return frame;
}

const Reading = struct { value: u32, timestamp_ms: u32 };

pub const Controller = struct {
    config: ProtocolConfig,
    state: State,
    fault: ?Fault = null,
    started_ms: u32,
    precharging_started_ms: ?u32 = null,
    bms: ?Reading = null,
    inverter: ?Reading = null,
    consecutive_qualifying: u8 = 0,

    pub fn init(config: ProtocolConfig, now_ms: u32) Controller {
        const valid = validConfig(config);
        return .{
            .config = config,
            .state = if (valid) .waiting_for_bms else .fault_latched,
            .fault = if (valid) null else .configuration,
            .started_ms = now_ms,
        };
    }

    pub fn ingest(self: *Controller, frame: Frame, now_ms: u32) void {
        if (self.state == .complete or self.state == .fault_latched) return;

        if (matches(frame, self.config.bms)) {
            const cart: bool = checkCart(frame, self.config.bms) catch {
                self.latch(.malformed_frame);
                return;
            };
            if (self.state == .waiting_for_bms and cart) {
                self.state = .complete;
                return;
            }
            const value = decodeVoltage(frame, self.config.bms) catch {
                self.latch(.malformed_frame);
                return;
            };
            self.bms = .{ .value = value, .timestamp_ms = now_ms };
            if (!self.plausible(value)) return self.latch(.implausible_voltage);
            if (self.state == .waiting_for_bms) {
                self.state = .precharging;
                self.precharging_started_ms = now_ms;
            }
        } else if (matches(frame, self.config.inverter)) {
            const value = decodeVoltage(frame, self.config.inverter) catch {
                self.latch(.malformed_frame);
                return;
            };
            self.inverter = .{ .value = value, .timestamp_ms = now_ms };
            if (!self.plausible(value)) return self.latch(.implausible_voltage);
            self.qualify(now_ms);
        }
    }

    pub fn tick(self: *Controller, now_ms: u32) void {
        if (self.state == .complete or self.state == .fault_latched) return;
        if (elapsed(now_ms, self.started_ms) >= self.config.precharge_timeout_ms)
            return self.latch(.timeout);

        if (self.state == .waiting_for_bms) {
            if (elapsed(now_ms, self.started_ms) >= self.config.freshness_timeout_ms)
                self.latch(.stale_bms);
            return;
        }

        const bms = self.bms orelse return self.latch(.stale_bms);
        if (elapsed(now_ms, bms.timestamp_ms) >= self.config.freshness_timeout_ms)
            return self.latch(.stale_bms);
        if (self.inverter) |reading| {
            if (elapsed(now_ms, reading.timestamp_ms) >= self.config.freshness_timeout_ms)
                return self.latch(.stale_inverter);
        } else if (elapsed(now_ms, self.precharging_started_ms.?) >= self.config.freshness_timeout_ms) {
            return self.latch(.stale_inverter);
        }
    }

    fn qualify(self: *Controller, now_ms: u32) void {
        const bms = self.bms orelse {
            self.consecutive_qualifying = 0;
            return;
        };
        const inverter = self.inverter.?;
        if (elapsed(now_ms, bms.timestamp_ms) >= self.config.freshness_timeout_ms) {
            self.latch(.stale_bms);
            return;
        }

        const lhs = @as(u64, inverter.value) * 100;
        const rhs = @as(u64, bms.value) * self.config.threshold_percent;
        if (lhs >= rhs) {
            self.consecutive_qualifying +|= 1; // saturating addition prevents wrapping to zero
            if (self.consecutive_qualifying >= self.config.qualifying_samples)
                self.state = .complete;
        } else {
            self.consecutive_qualifying = 0;
        }
    }

    fn plausible(self: *Controller, value: u32) bool {
        return value >= self.config.min_voltage and value <= self.config.max_voltage;
    }

    fn latch(self: *Controller, fault: Fault) void {
        self.state = .fault_latched;
        self.fault = fault;
    }
};

fn matches(frame: Frame, spec: VoltageSpec) bool {
    return frame.id == spec.id and frame.kind == spec.kind;
}

fn validConfig(config: ProtocolConfig) bool {
    return config.ready and
        validSpec(config.bms) and
        validSpec(config.inverter) and
        !sameFrame(config.bms, config.inverter) and
        config.min_voltage <= config.max_voltage and
        config.threshold_percent != 0 and
        config.threshold_percent <= 100 and
        config.qualifying_samples != 0 and
        config.freshness_timeout_ms != 0 and
        config.precharge_timeout_ms != 0 and
        config.max_voltage <= std.math.maxInt(u16);
}

fn sameFrame(a: VoltageSpec, b: VoltageSpec) bool {
    return a.id == b.id and a.kind == b.kind;
}

fn validSpec(spec: VoltageSpec) bool {
    const width: usize = switch (spec.width) {
        .one => 1,
        .two => 2,
        .four => 4,
    };
    const max_id: u32 = switch (spec.kind) {
        .standard => 0x7FF,
        .extended => 0x1FFF_FFFF,
    };
    return spec.id <= max_id and
        @as(usize, spec.offset) + width <= 8 and
        spec.divisor != 0;
}

fn elapsed(now: u32, then: u32) u32 {
    return now -% then;
}

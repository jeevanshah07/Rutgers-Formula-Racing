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
};

pub const ProtocolConfig = struct {
    ready: bool,
    bitrate: u32,
    bms: VoltageSpec,
    inverter: VoltageSpec,
    min_voltage: u32 = 1,
    max_voltage: u32 = 390,
    threshold_percent: u8 = 90,
    qualifying_samples: u8 = 1,
    freshness_timeout_ms: u32 = 100_000,
    precharge_timeout_ms: u32 = 100_000,
};

pub const placeholder_config = ProtocolConfig{
    .ready = true, // TODO: verify values and change to true
    .bitrate = 500_000,
    .bms = .{ .id = 0x01, .kind = .extended, .offset = 0, .width = .two, .endian = .little, .multiplier = 10 },
    .inverter = .{ .id = 0x02, .kind = .extended, .offset = 0, .width = .two, .endian = .little, .multiplier = 10 },
    .min_voltage = 1,
    .max_voltage = 390,
};

pub const DecodeError = error{
    RemoteFrame,
    WrongFrame,
    PayloadTooShort,
    InvalidScale,
    Overflow,
};

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
pub const Fault = enum { configuration, malformed_frame, implausible_voltage, stale_data, timeout };

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
        return .{
            .config = config,
            .state = if (config.ready) .waiting_for_bms else .fault_latched,
            .fault = if (config.ready) null else .configuration,
            .started_ms = now_ms,
        };
    }

    pub fn ingest(self: *Controller, frame: Frame, now_ms: u32) void {
        if (self.state == .complete or self.state == .fault_latched) return;

        if (matches(frame, self.config.bms)) {
            const value = decodeVoltage(frame, self.config.bms) catch {
                self.latch(.malformed_frame);
                return;
            };
            if (!self.plausible(value)) return self.latch(.implausible_voltage);
            self.bms = .{ .value = value, .timestamp_ms = now_ms };
            if (self.state == .waiting_for_bms) {
                self.state = .precharging;
                self.precharging_started_ms = now_ms;
            }
        } else if (matches(frame, self.config.inverter)) {
            const value = decodeVoltage(frame, self.config.inverter) catch {
                self.latch(.malformed_frame);
                return;
            };
            if (!self.plausible(value)) return self.latch(.implausible_voltage);
            self.inverter = .{ .value = value, .timestamp_ms = now_ms };
            self.qualify(now_ms);
        }
    }

    pub fn tick(self: *Controller, now_ms: u32) void {
        if (self.state == .complete or self.state == .fault_latched) return;
        if (elapsed(now_ms, self.started_ms) >= self.config.precharge_timeout_ms)
            return self.latch(.timeout);

        if (self.state == .waiting_for_bms) {
            if (elapsed(now_ms, self.started_ms) >= self.config.freshness_timeout_ms)
                self.latch(.stale_data);
            return;
        }

        const bms = self.bms orelse return self.latch(.stale_data);
        if (elapsed(now_ms, bms.timestamp_ms) >= self.config.freshness_timeout_ms)
            return self.latch(.stale_data);
        if (self.inverter) |reading| {
            if (elapsed(now_ms, reading.timestamp_ms) >= self.config.freshness_timeout_ms)
                return self.latch(.stale_data);
        } else if (elapsed(now_ms, self.precharging_started_ms.?) >= self.config.freshness_timeout_ms) {
            return self.latch(.stale_data);
        }
    }

    fn qualify(self: *Controller, now_ms: u32) void {
        const bms = self.bms orelse {
            self.consecutive_qualifying = 0;
            return;
        };
        const inverter = self.inverter.?;
        if (elapsed(now_ms, bms.timestamp_ms) >= self.config.freshness_timeout_ms) {
            self.latch(.stale_data);
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

fn elapsed(now: u32, then: u32) u32 {
    return now -% then;
}

fn testConfig() ProtocolConfig {
    var config = placeholder_config;
    config.ready = true;
    config.bms = .{ .id = 0x100, .kind = .standard, .offset = 1, .width = .two, .endian = .big };
    config.inverter = .{ .id = 0x101, .kind = .standard, .offset = 1, .width = .two, .endian = .big };
    config.min_voltage = 100;
    config.max_voltage = 1_000;
    return config;
}

fn testFrame(id: u32, value: u16) Frame {
    var frame = Frame{ .id = id, .kind = .standard, .dlc = 3, .data = @splat(0) };
    std.mem.writeInt(u16, frame.data[1..3], value, .big);
    return frame;
}

test "placeholder configuration latches safe fault" {
    const controller = Controller.init(placeholder_config, 0);
    try std.testing.expectEqual(State.fault_latched, controller.state);
    try std.testing.expectEqual(Fault.configuration, controller.fault.?);
}

test "decode validates frame and payload" {
    const config = testConfig();
    try std.testing.expectEqual(@as(u32, 500), try decodeVoltage(testFrame(0x100, 500), config.bms));
    var short = testFrame(0x100, 500);
    short.dlc = 2;
    try std.testing.expectError(error.PayloadTooShort, decodeVoltage(short, config.bms));
    var remote = testFrame(0x100, 500);
    remote.remote = true;
    try std.testing.expectError(error.RemoteFrame, decodeVoltage(remote, config.bms));
}

test "three fresh samples complete precharge" {
    var controller = Controller.init(testConfig(), 0);
    controller.ingest(testFrame(0x100, 500), 10);
    controller.ingest(testFrame(0x101, 449), 20);
    try std.testing.expectEqual(@as(u8, 0), controller.consecutive_qualifying);
    controller.ingest(testFrame(0x101, 450), 30);
    controller.ingest(testFrame(0x101, 460), 40);
    try std.testing.expectEqual(State.precharging, controller.state);
    controller.ingest(testFrame(0x101, 470), 50);
    try std.testing.expectEqual(State.complete, controller.state);
}

test "non-qualifying sample resets sequence" {
    var controller = Controller.init(testConfig(), 0);
    controller.ingest(testFrame(0x100, 500), 0);
    controller.ingest(testFrame(0x101, 450), 10);
    controller.ingest(testFrame(0x101, 440), 20);
    controller.ingest(testFrame(0x101, 450), 30);
    controller.ingest(testFrame(0x101, 450), 40);
    try std.testing.expectEqual(State.precharging, controller.state);
}

test "stale data and total timeout latch faults" {
    var stale = Controller.init(testConfig(), 0);
    stale.ingest(testFrame(0x100, 500), 0);
    stale.tick(1_000);
    try std.testing.expectEqual(Fault.stale_data, stale.fault.?);

    var timeout_config = testConfig();
    timeout_config.freshness_timeout_ms = 20_000;
    var timed_out = Controller.init(timeout_config, 0);
    timed_out.tick(10_000);
    try std.testing.expectEqual(Fault.timeout, timed_out.fault.?);
}

test "missing inverter data becomes stale even while BMS updates" {
    var controller = Controller.init(testConfig(), 0);
    controller.ingest(testFrame(0x100, 500), 100);
    controller.ingest(testFrame(0x100, 500), 1_000);
    controller.tick(1_100);
    try std.testing.expectEqual(Fault.stale_data, controller.fault.?);
}

test "malformed and implausible matching frames latch faults" {
    var malformed = Controller.init(testConfig(), 0);
    var short = testFrame(0x100, 500);
    short.dlc = 1;
    malformed.ingest(short, 0);
    try std.testing.expectEqual(Fault.malformed_frame, malformed.fault.?);

    var implausible = Controller.init(testConfig(), 0);
    implausible.ingest(testFrame(0x100, 99), 0);
    try std.testing.expectEqual(Fault.implausible_voltage, implausible.fault.?);
}

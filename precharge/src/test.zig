const std = @import("std");
const logic = @import("root.zig");

fn testConfig() logic.ProtocolConfig {
    var config = logic.base_config;
    config.ready = true;
    config.bms = .{ .id = 0x100, .kind = .standard, .offset = 1, .width = .two, .endian = .big };
    config.inverter = .{ .id = 0x101, .kind = .standard, .offset = 1, .width = .two, .endian = .big };
    config.min_voltage = 100;
    config.max_voltage = 1_000;
    config.freshness_timeout_ms = 1_000;
    config.precharge_timeout_ms = 10_000;
    return config;
}

fn testFrame(id: u32, value: u16) logic.Frame {
    var frame = logic.Frame{ .id = id, .kind = .standard, .dlc = 3, .data = @splat(0) };
    std.mem.writeInt(u16, frame.data[1..3], value, .big);
    return frame;
}

test "base configuration is enabled" {
    const controller = logic.Controller.init(logic.base_config, 0);
    try std.testing.expectEqual(logic.State.waiting_for_bms, controller.state);
    try std.testing.expectEqual(@as(?logic.Fault, null), controller.fault);
}

test "fault codes and voltages map to the info CAN frame" {
    try std.testing.expectEqual(@as(u8, 1), @intFromEnum(logic.Fault.stale_bms));
    try std.testing.expectEqual(@as(u8, 2), @intFromEnum(logic.Fault.stale_inverter));
    try std.testing.expectEqual(@as(u8, 3), @intFromEnum(logic.Fault.configuration));
    try std.testing.expectEqual(@as(u8, 4), @intFromEnum(logic.Fault.malformed_frame));
    try std.testing.expectEqual(@as(u8, 5), @intFromEnum(logic.Fault.implausible_voltage));
    try std.testing.expectEqual(@as(u8, 6), @intFromEnum(logic.Fault.timeout));

    var controller = logic.Controller.init(testConfig(), 0);
    controller.ingest(testFrame(0x100, 500), 0);
    controller.ingest(testFrame(0x101, 450), 1);
    controller.tick(1_000);
    const frame = logic.infoFrame(controller);
    try std.testing.expectEqual(@as(u32, 0x0D3), frame.id);
    try std.testing.expectEqual(logic.FrameKind.standard, frame.kind);
    try std.testing.expectEqual(@as(u4, 5), frame.dlc);
    try std.testing.expectEqual(@as(u8, 1), frame.data[0]);
    try std.testing.expectEqual(@as(u16, 500), std.mem.readInt(u16, frame.data[1..3], .big));
    try std.testing.expectEqual(@as(u16, 450), std.mem.readInt(u16, frame.data[3..5], .big));
}

test "decode validates frame and payload" {
    const config = testConfig();
    try std.testing.expectEqual(@as(u32, 500), try logic.decodeVoltage(testFrame(0x100, 500), config.bms));
    var short = testFrame(0x100, 500);
    short.dlc = 2;
    try std.testing.expectError(error.PayloadTooShort, logic.decodeVoltage(short, config.bms));
    var remote = testFrame(0x100, 500);
    remote.remote = true;
    try std.testing.expectError(error.RemoteFrame, logic.decodeVoltage(remote, config.bms));
}

test "cart detection is optional and validates payload" {
    const config = testConfig();
    try std.testing.expect(!try logic.checkCart(testFrame(0x100, 500), config.bms));

    var cart_spec = config.bms;
    cart_spec.cartOffset = 5;
    var cart_frame = testFrame(0x100, 500);
    try std.testing.expectError(error.PayloadTooShort, logic.checkCart(cart_frame, cart_spec));
    cart_frame.dlc = 6;
    cart_frame.data[5] = 0;
    try std.testing.expect(try logic.checkCart(cart_frame, cart_spec));
    cart_frame.data[5] = 1;
    try std.testing.expect(!try logic.checkCart(cart_frame, cart_spec));
    cart_frame.data[5] = 2;
    try std.testing.expectError(error.InvalidCartFlag, logic.checkCart(cart_frame, cart_spec));
}

test "invalid configuration latches safe fault" {
    var config = testConfig();
    config.qualifying_samples = 0;
    var controller = logic.Controller.init(config, 0);
    try std.testing.expectEqual(logic.Fault.configuration, controller.fault.?);

    config = testConfig();
    config.bms.divisor = 0;
    controller = logic.Controller.init(config, 0);
    try std.testing.expectEqual(logic.Fault.configuration, controller.fault.?);

    config = testConfig();
    config.bms.id = 0x800;
    controller = logic.Controller.init(config, 0);
    try std.testing.expectEqual(logic.Fault.configuration, controller.fault.?);

    config = testConfig();
    config.inverter.id = config.bms.id;
    controller = logic.Controller.init(config, 0);
    try std.testing.expectEqual(logic.Fault.configuration, controller.fault.?);

    config = testConfig();
    config.max_voltage = std.math.maxInt(u16) + 1;
    controller = logic.Controller.init(config, 0);
    try std.testing.expectEqual(logic.Fault.configuration, controller.fault.?);
}

test "three fresh samples complete precharge" {
    var controller = logic.Controller.init(testConfig(), 0);
    controller.ingest(testFrame(0x100, 500), 10);
    controller.ingest(testFrame(0x101, 449), 20);
    try std.testing.expectEqual(@as(u8, 0), controller.consecutive_qualifying);
    controller.ingest(testFrame(0x101, 450), 30);
    controller.ingest(testFrame(0x101, 460), 40);
    try std.testing.expectEqual(logic.State.precharging, controller.state);
    controller.ingest(testFrame(0x101, 470), 50);
    try std.testing.expectEqual(logic.State.complete, controller.state);
}

test "non-qualifying sample resets sequence" {
    var controller = logic.Controller.init(testConfig(), 0);
    controller.ingest(testFrame(0x100, 500), 0);
    controller.ingest(testFrame(0x101, 450), 10);
    controller.ingest(testFrame(0x101, 440), 20);
    controller.ingest(testFrame(0x101, 450), 30);
    controller.ingest(testFrame(0x101, 450), 40);
    try std.testing.expectEqual(logic.State.precharging, controller.state);
}

test "stale BMS data and total timeout latch faults" {
    var stale = logic.Controller.init(testConfig(), 0);
    stale.ingest(testFrame(0x100, 500), 0);
    stale.tick(1_000);
    try std.testing.expectEqual(logic.Fault.stale_bms, stale.fault.?);

    var timeout_config = testConfig();
    timeout_config.freshness_timeout_ms = 20_000;
    var timed_out = logic.Controller.init(timeout_config, 0);
    timed_out.tick(10_000);
    try std.testing.expectEqual(logic.Fault.timeout, timed_out.fault.?);
}

test "missing inverter data becomes stale even while BMS updates" {
    var controller = logic.Controller.init(testConfig(), 0);
    controller.ingest(testFrame(0x100, 500), 100);
    controller.ingest(testFrame(0x100, 500), 1_000);
    controller.tick(1_100);
    try std.testing.expectEqual(logic.Fault.stale_inverter, controller.fault.?);
}

test "malformed and implausible matching frames latch faults" {
    var malformed = logic.Controller.init(testConfig(), 0);
    var short = testFrame(0x100, 500);
    short.dlc = 1;
    malformed.ingest(short, 0);
    try std.testing.expectEqual(logic.Fault.malformed_frame, malformed.fault.?);

    var implausible = logic.Controller.init(testConfig(), 0);
    implausible.ingest(testFrame(0x100, 99), 0);
    try std.testing.expectEqual(logic.Fault.implausible_voltage, implausible.fault.?);
}

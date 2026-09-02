//! RFR27 bench-safe precharge controller.

const std = @import("std");
const microzig = @import("microzig");
const logic = @import("root.zig");
const stm32 = microzig.hal;
const peripherals = microzig.chip.peripherals;
const rcc = stm32.rcc;
const gpio = stm32.gpio;
const time = stm32.time;

pub const microzig_options: microzig.Options = .{ .interrupts = .{} };
pub const panic = microzig.panic;
pub const std_options = microzig.std_options(.{});

comptime {
    _ = microzig.export_startup();
}

const protocol = logic.base_config;
const loop_period_ms: u32 = 5;

const pins = struct {
    const enable = gpio.Pin.from_port(.A, 7);
    const can_rx = gpio.Pin.from_port(.A, 11);
    const can_tx = gpio.Pin.from_port(.A, 12);
};

fn systemClockConfig() !void {
    _ = try rcc.apply(.{
        .SYSCLKSource = .PLL1_P,
        .PLLSourceVirtual = .HSI_Div2,
        .PLLMUL = .Mul9,
        .AHBCLKDivider = .Div1,
        .APB1CLKDivider = .Div1,
        .ADCPresc = .Div6,
    });
}

fn packedFilterId(spec: logic.VoltageSpec) u32 {
    return switch (spec.kind) {
        .extended => ((spec.id & 0x1FFF_FFFF) << 3) | (1 << 2),
        .standard => (spec.id & 0x7FF) << 21,
    };
}

fn canInit() void {
    const can = peripherals.CAN;

    // Enter initialization mode before changing bit timing.
    can.MCR.modify(.{ .SLEEP = 0, .INRQ = 1 });
    while (can.MSR.read().INAK == 0) {}

    // APB1 is 36 MHz. BRP=4, 18 time quanta/bit gives 500 kbit/s;
    // register fields contain each value minus one.
    if (protocol.bitrate != 500_000)
        @compileError("Update bxCAN bit timing before selecting another bitrate");
    can.BTR.modify(.{
        .BRP = 3,
        .@"TS[0]" = 14,
        .@"TS[1]" = 1,
        .SJW = 0,
        .LBKM = 0,
        .SILM = .Normal,
    });

    can.FMR.modify(.{ .FINIT = 1 });
    can.FM1R.raw |= 1; // bank 0, identifier-list mode
    can.FS1R.raw |= 1; // bank 0, 32-bit scale
    can.FFA1R.raw &= ~@as(u32, 1); // bank 0 -> FIFO0
    can.FB[0].FR1.raw = packedFilterId(protocol.bms);
    can.FB[0].FR2.raw = packedFilterId(protocol.inverter);
    can.FA1R.raw |= 1;
    can.FMR.modify(.{ .FINIT = 0 });

    can.MCR.modify(.{ .INRQ = 0 });
    while (can.MSR.read().INAK == 1) {}
}

fn canReceive() ?logic.Frame {
    const can = peripherals.CAN;
    if (can.RFR[0].read().FMP == 0) return null;

    const fifo = &can.RX[0];
    const rir = fifo.RIR.read();
    var frame = logic.Frame{
        .id = undefined,
        .kind = if (rir.IDE == .Extended) .extended else .standard,
        .remote = rir.RTR == .Remote,
        .dlc = fifo.RDTR.read().DLC,
        .data = undefined,
    };
    frame.id = switch (frame.kind) {
        .extended => (@as(u32, rir.STID) << 18) | rir.EXID,
        .standard => rir.STID,
    };
    std.mem.writeInt(u32, frame.data[0..4], fifo.RDLR.raw, .little);
    std.mem.writeInt(u32, frame.data[4..8], fifo.RDHR.raw, .little);

    can.RFR[0].modify(.{ .RFOM = 1 });
    return frame;
}

fn canTransmit(frame: logic.Frame) bool {
    const can = peripherals.CAN;
    const tsr = can.TSR.read();
    const index: u2 = if (tsr.@"TME[0]" == 1)
        0
    else if (tsr.@"TME[1]" == 1)
        1
    else if (tsr.@"TME[2]" == 1)
        2
    else
        return false;
    const mailbox = &can.TX[index];

    mailbox.TIR.raw = switch (frame.kind) {
        .standard => (frame.id & 0x7FF) << 21,
        .extended => ((frame.id & 0x1FFF_FFFF) << 3) | (1 << 2),
    };
    mailbox.TDTR.modify(.{ .DLC = frame.dlc });
    mailbox.TDLR.raw = std.mem.readInt(u32, frame.data[0..4], .little);
    mailbox.TDHR.raw = std.mem.readInt(u32, frame.data[4..8], .little);
    mailbox.TIR.modify(.{ .TXRQ = 1 });
    return true;
}

pub fn main() !void {
    try systemClockConfig();

    rcc.enable_clock(.GPIOA);
    rcc.enable_clock(.AFIO);
    rcc.enable_clock(.CAN);
    rcc.enable_clock(.TIM2);

    // Establish the safe output state before initializing communications.
    pins.enable.put(0);
    pins.enable.set_output_mode(.general_purpose_push_pull, .max_2MHz);
    pins.can_rx.set_input_mode(.floating);
    pins.can_tx.set_output_mode(.alternate_function_push_pull, .max_50MHz);

    time.init_timer(.TIM2);
    canInit();

    var now_ms: u32 = 0;
    var controller = logic.Controller.init(protocol, now_ms);

    while (true) {
        while (canReceive()) |frame| controller.ingest(frame, now_ms);
        controller.tick(now_ms);
        var info_sent = canTransmit(logic.infoFrame(controller.fault, controller));

        switch (controller.state) {
            .complete => {
                pins.enable.put(1);
                while (true) time.sleep_ms(1_000);
            },
            .fault_latched => {
                pins.enable.put(0);

                while (!info_sent) {
                    time.sleep_ms(loop_period_ms);
                    info_sent = canTransmit(logic.infoFrame(controller.fault, controller));
                }
                while (true) time.sleep_ms(1_000);
            },
            else => {},
        }

        time.sleep_ms(loop_period_ms);
        now_ms +%= loop_period_ms; // leverage wrapping addition to keep the timer going
    }
}

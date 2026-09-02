const std = @import("std");
const microzig = @import("microzig");

const MicroBuild = microzig.MicroBuild(.{
    .stm32 = true,
});

pub fn build(b: *std.Build) void {
    const optimize = b.standardOptimizeOption(.{});

    const mz_dep = b.dependency("microzig", .{});
    const mb = MicroBuild.init(b, mz_dep) orelse return;
    const stm32 = mb.ports.stm32;

    const fw = mb.add_firmware(.{
        .name = "rfr-27-precharge",
        .target = stm32.chips.STM32F103T8,
        .optimize = optimize,
        .root_source_file = b.path("src/main.zig"),
    });

    mb.install_firmware(fw, .{});

    // For debugging, we also always install the firmware as an ELF file
    mb.install_firmware(fw, .{ .format = .elf });
    mb.install_firmware(fw, .{ .format = .binary });

    const unit_tests = b.addTest(.{
        .root_module = b.createModule(.{
            .root_source_file = b.path("src/test.zig"),
            .target = b.graph.host,
        }),
    });
    const run_unit_tests = b.addRunArtifact(unit_tests);
    const test_step = b.step("test", "Run host-side protocol and state-machine tests");
    test_step.dependOn(&run_unit_tests.step);
}

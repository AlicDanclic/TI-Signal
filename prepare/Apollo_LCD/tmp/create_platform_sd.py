import os
import vitis

workspace = r"E:\Apollo_LCD\vitis_sd_workspace"
xsa = r"E:\Apollo_LCD\zynq_boot_support\mizar_ps_wrapper_sd.xsa"

client = vitis.create_client()
client.set_workspace(path=workspace)
platform = client.create_platform_component(
    name="platform_sd",
    hw_design=xsa,
    os="standalone",
    cpu="ps7_cortexa9_0",
    domain_name="standalone_ps7_cortexa9_0",
    template="empty_application",
    generate_dtb=False,
    desc="Mizar Z7020 SD boot support platform"
)
platform.build()
print("PLATFORM_SD_BUILD_DONE")
vitis.dispose()

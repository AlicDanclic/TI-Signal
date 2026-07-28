# 2026-07-14T22:11:23.973309500
import vitis

client = vitis.create_client()
client.set_workspace(path="vitis_sd_workspace")

platform = client.create_platform_component(name = "platform_sd",hw_design = "$COMPONENT_LOCATION/../../zynq_boot_support/mizar_ps_wrapper_sd.xsa",desc = "Mizar Z7020 SD boot support platform",os = "standalone",cpu = "ps7_cortexa9_0",domain_name = "standalone_ps7_cortexa9_0",template = "empty_application",generate_dtb = False)

vitis.dispose()


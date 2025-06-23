import os
import vitis

pwd = os.getcwd()

client = vitis.create_client()
client.update_workspace(pwd)
client.set_workspace(path=pwd)

platform = client.create_platform_component(
    name = "GeRM-192-384",
	hw_design = pwd + "/xsa/germ-192-384.xsa",
	os = "freertos",
	cpu = "ps7_cortexa9_0",
	domain_name = "freertos_ps7_cortexa9_0"
)

domain = platform.get_domain(name="freertos_ps7_cortexa9_0")

domain.set_lib(
    lib_name="lwip220",
    path = os.environ.get('XILINX_VITIS') + "/data/embeddedsw/ThirdParty/sw_services/lwip220_v1_1"
)

status = domain.set_config(option = "lib", param = "lwip220_api_mode", value = "SOCKET_API", lib_name="lwip220")

status = domain.set_lib(lib_name="xilffs", path="/opt/Xilinx/Vitis/2024.2/data/embeddedsw/lib/sw_services/xilffs_v5_3")

status = domain.set_config(option = "lib", param = "XILFFS_read_only", value = "true", lib_name="xilffs")


#bsp_path = os.path.join(platform.directory, "freertos_ps7_cortexa9_0")
#settings_mk = os.path.join(bsp_path, "settings.mk")
#
#with open(settings_mk, "a") as f:
#    print( "Revising settings.mk" )
#    f.write("\nLWIP_SOCKET = 1\n")
#
#domain.build()  # Rebuilds the BSP with new settings

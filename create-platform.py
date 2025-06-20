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


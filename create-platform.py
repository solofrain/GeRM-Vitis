import vitis

client = vitis.create_client()

client.update_workspace("/data/work/fpga/prj/germ-vitis")
client.set_workspace(path="/data/work/fpga/prj/germ-vitis")

platform = client.create_platform_component(
    name = "GeRM-192-384",
	hw_design = "$COMPONENT_LOCATION/../xsa/germ-192-384.xsa",
	os = "freertos",
	cpu = "ps7_cortexa9_0",
	domain_name = "freertos_ps7_cortexa9_0"
)

platform = client.get_component(name="GeRM-192-384")
status = platform.build()


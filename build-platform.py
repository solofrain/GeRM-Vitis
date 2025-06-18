import vitis

client = vitis.create_client()

client.set_workspace(path="/data/work/fpga/prj/germ-vitis")

platform = client.get_component(name="GeRM-192-384")
status = platform.build()


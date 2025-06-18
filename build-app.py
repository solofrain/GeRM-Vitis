import vitis

client = vitis.create_client()

client.set_workspace(path="/data/work/fpga/prj/germ-vitis")

app = client.get_component(name="ZynqDetector")
status = app.build()


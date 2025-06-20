import os
import vitis

pwd = os.getcwd()

client = vitis.create_client()
client.set_workspace(path=pwd)

app = client.get_component(name="ZynqDetector")
status = app.build()


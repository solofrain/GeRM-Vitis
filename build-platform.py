import os
import vitis

pwd = os.getcwd()

client = vitis.create_client()
client.set_workspace(path=pwd)

platform = client.get_component(name="GeRM-192-384")
status = platform.build()


import os
import vitis

pwd = os.getcwd()

client = vitis.create_client()
client.set_workspace(path=pwd)

#platform = client.get_component(name="GeRM-192-384")

comp = client.create_app_component(
    name="ZynqDetector",
    platform="$COMPONENT_LOCATION/../GeRM-192-384/export/GeRM-192-384/GeRM-192-384.xpfm",
    domain="freertos_ps7_cortexa9_0"
)

print("App ZynqDetector created.")

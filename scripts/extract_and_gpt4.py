import os
import subprocess


def extract_and_call_script(folder_path, interval=1250, script_path="/lab/tmpig10c/henghui/vision_gpt4.py"):
    # Check if the folder exists
    img_path = folder_path + 'all_imgs/'
    if not os.path.exists(img_path):
        print(f"Error: Folder '{folder_path}' not found.")
        return

    files = os.listdir(img_path)
    files.sort()

    for i, file_name in enumerate(files, start=1):
        if i % interval == 0:
            image_path = os.path.join(img_path, file_name)
            print(f"Processing image: {image_path}")

            # Call the other script with the image as a parameter
            subprocess.run(["python", script_path, image_path, folder_path + 'text_gpt4/'])


if __name__ == "__main__":
    current_path = os.getcwd()
    target_path = os.path.abspath(os.path.join(current_path, "../.."))

    # Specify the folder containing images and the path to the other script
    folder_path = "/lab/tmpig23b/navisim/data/bag_dump/2023_03_11/0/"
    script_path = f'{target_path}/3d2d_ann/gpt4/vision_gpt4.py'

    # # Call the function with the specified parameters
    extract_and_call_script(folder_path, interval=1250, script_path=script_path)


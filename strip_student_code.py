import os
import re

def strip_student_code(base_dir):
    for root, dirs, files in os.walk(base_dir):
        for filename in files:
            if filename.endswith(".py"):
                file_path = os.path.join(root, filename)
                with open(file_path, 'r') as f:
                    lines = f.readlines()

                new_lines = []
                skip = False
                for line in lines:
                    if "STUDENT CODE HERE" in line:
                    if "END STUDENT CODE" in line:
                        skip = False
                        new_lines.append(line.rstrip() + "\n")  # Keep the marker
                        continue
                    if not skip:
                        new_lines.append(line)

                with open(file_path, 'w') as f:
                    f.writelines(new_lines)

                print(f"Stripped student code in {file_path}")

if __name__ == "__main__":
    strip_student_code(".")

import os
import re

def strip_student_code(path):
    for root, dirs, files in os.walk(path):
        for fname in files:
            if fname.endswith(".py"):
                fpath = os.path.join(root, fname)
                with open(fpath, "r") as f:
                    content = f.read()

                # Regex to remove everything between markers (non-greedy)
                new_content = re.sub(
                    r"# STUDENT CODE HERE

        # END STUDENT CODE", 
                    "# STUDENT CODE HERE

        # END STUDENT CODE", 
                    content, 
                    flags=re.DOTALL
                )

                if new_content != content:
                    print(f"Stripped: {fpath}")
                    with open(fpath, "w") as f:
                        f.write(new_content)

if __name__ == "__main__":
    strip_student_code(".")

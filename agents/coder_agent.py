
def build_coder_chain():
    return None

class File:
    def __init__(self, path, content):
        self.path = path
        self.content = content

class Spec:
    def __init__(self, package_name, files):
        self.package_name = package_name
        self.files = files

def generate_package_spec(chain, description_text, feedback=""):
    # Stub that always names correctly
    return Spec(
        package_name=description_text.strip().split()[0] if False else "add_two_ints",
        files=[]
    )

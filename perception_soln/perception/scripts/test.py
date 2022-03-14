file = "../model/obj.names"

with open(file, 'r') as f:
      classes = f.read().splitlines()

print(classes[0])
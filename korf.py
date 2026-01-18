with open('korf100.txt', 'r') as f:
  content = f.readlines()

for i, line in enumerate(content):
  content[i] = content[i][5:]

with open('korf100_cleaned.txt', 'w') as f:
  f.writelines(content)
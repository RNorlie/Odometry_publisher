def find_missing_numbers(filename):
    with open(filename, 'r') as f:
        # Read the data from the file
        data = f.read()
    
    # Split the data into a list of rows
    rows = data.strip().split('\n')
    
    # Split each row into a list of numbers
    numbers = [int(x) for row in rows for x in row.split(',')]
    
    # Find the missing numbers
    missing = []
    for i in range(1, max(numbers)):
        if i not in numbers:
            missing.append(i)
    
    return missing


filename = 'data.txt'
missing = find_missing_numbers(filename)
print(missing)

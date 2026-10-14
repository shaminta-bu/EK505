'''
Utilities for EK505 assignments
'''

def print_matrix(matrix, alignment='right'):
    """
    Prints a SymPy Matrix or NumPy array consistently by converting all elements
    to strings.

    Parameters:
    - matrix: A SymPy Matrix/Array or NumPy ndarray (1D or 2D).
    - alignment: 'right', 'left', or 'center' alignment for the columns.
    """
    # Ensure the input has a shape attribute (standard for NumPy and SymPy)
    if not hasattr(matrix, 'shape'):
        raise TypeError("Input must have a 'shape' attribute (e.g., SymPy Matrix or NumPy array).")

    shape = matrix.shape
    if len(shape) == 1:
        rows, cols = 1, shape[0]
        is_1d = True
    elif len(shape) == 2:
        rows, cols = shape
        is_1d = False
    else:
        raise ValueError("Only 1D or 2D arrays/matrices are supported.")

    # Part 1: Convert all elements to strings and find the maximum length
    string_grid = []
    max_len = 0

    for r in range(rows):
        row_strings = []
        for c in range(cols):
            # Handle indexing differences between 1D and 2D
            val = matrix[c] if is_1d else matrix[r, c]

            # Convert to string and track the maximum length
            val_str = str(val)
            row_strings.append(val_str)
            max_len = max(max_len, len(val_str))

        string_grid.append(row_strings)

    # Part 2: Print the array using max_len as the column width
    for row in string_grid:
        formatted_row = []
        for s in row:
            # Align the string based on user preference
            if alignment == 'left':
                formatted_row.append(s.ljust(max_len))
            elif alignment == 'center':
                formatted_row.append(s.center(max_len))
            else:
                formatted_row.append(s.rjust(max_len))

        # Join elements with a couple of spaces and print
        row_str = "  ".join(formatted_row)
        print(f"  [ {row_str} ]")

import random

numbers = [random.randint(-10, 10) for x in range (10)]
print(f"Zufällige Zahlen: {numbers}")
print(f"Summe: {sum(numbers)}")
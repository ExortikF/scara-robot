# 1
# word = input().lower()
# target = 'prime'
# u = 0
# answ = 0
# for i in range(len(word)):
    
#     if word[i] == target[u]:
#         u += 1
        
#     if u == 5:
#             u = 0
#             answ += 1
#     # print(word[:i+1], u)

# print(answ)

# 3
# l = list(map(int, input().split()))
# a = l[0]
# b = l[1]

# if a <= b:
#     for i in range(a, b+1):
#         print(i)
# else:
#     for i in range(a, b-1, -1):
#         print(i)

# 4

# c = list(map(float, input().split()))
# print( abs(c[0] - c[1]) )

# 5 
from math import gcd
n = int(input())
nums = []
for i in range(2, n+1):
    for j in range(1, i+1):
        
        if gcd(i, j) == 1:
            nums.append((j, i, j/i))

nums.sort(key = lambda x: x[2])
for i in nums:
    print(f'{i[0]}/{i[1]}')
import os 

fd1, fd2 = os.openpty()
print(fd1)
print(fd2)
print(os.ttyname(fd1))
print(os.ttyname(fd2))



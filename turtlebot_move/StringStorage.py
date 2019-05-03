# The text is to be stored in this string
x = ''

def storeString(inString):
    global x
    global file
    file = open('Acovariables.txt','a')
    x = inString
    # Do something with the string

    file.write(inString + '\n') 
    file.close()
    print(inString)
    return

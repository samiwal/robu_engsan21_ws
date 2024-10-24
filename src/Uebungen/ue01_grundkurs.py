# Kommentare: # -> Kommentar
# Oder strg+shift+7
# Docstring
"""
Kann auch als Kommentar verwendet werden
"""

import math
from math import pi # as M_PI
import matplotlib.pyplot as plt #Bibliothek für Diagramme

#Variablen müssen mit abc oder _ beginnen
#Keine umlaute
#darf kein python-schlüsselwort sein
titel = "Datentypen"
vorname = "Samuel"
nachname = "Engel"
name = vorname +" "+nachname
Klassengott = True
Direktor = False
groesse = 1.74
schuhgroesse = 41
impedanz = 12 + 24j
# print("Ausgabe von unterschiedlichen",titel)
print(f"Ausgabe von unterschiedlichen {3*(titel + ' ')}")
print(f"Mein Name ist {name}")
print(Klassengott, type(Klassengott))
print(groesse, type(groesse))
print(schuhgroesse, type(schuhgroesse))
print(impedanz, type(impedanz))
print("Impedanzberechnung von einem RC-Glied")
print("=====================================")

resistor = 1e3
capacitor = 1e-6
frequenzy = 50.0

impedanz = complex(resistor, 1/(2*pi*frequenzy*capacitor))
tau = resistor * capacitor
print(f"Impedanz = {impedanz/1000:<10.2} kOhm")
print(f"Zeitkonstante = {tau*1000:<10.2} ms")
print(f"Grenzfrequenz = {1/(2*pi*tau):<10.2} Hz")

myList = [1, 2, -5, pi, name, impedanz, [7, 8]]
print(myList[0])
print(myList[-1][1])
print(myList[2:4])
print(myList[-2:])
print(myList[:5:2])
print(myList[::2])
myList += [3]
myList.append(3)
print(myList[-2:])
a, b, c = myList[:3]
print(a,b,c)
for i in range(2,10,2):
    print(i)
x = [1,4,6,4,7,3,1]
x_set = set(x)
print("x:",x)
print("x_set:",x_set)

#dictionaris
print("Datentyp dict")
print("=============")

person = {"vorname": vorname,
          "nachname": nachname,
          "größe": groesse,
          "kg": Klassengott,
          "zustand":"jung"}
print(person["vorname"],person["nachname"], "ist unser Kg und sieht", person["zustand"] ,"aus")
print("person.keys(): ",person.keys())
print("person.values(): ",person.values())
print("person.items(): ",person.items())
print("schleifen")
print("=========")
#y(x) = x²
y = []
for x in range(10):
    y.append(x**2)
print (y)

#y = [x**2 for x in range(10)]

plt.plot(range(10),y) 
plt.ylabel("y(x)=x²")
plt.xlabel("x")
# plt.show()

words = ["Apfel","Banane","Kirsche","Pfirsich","Traube"]
longest_word = " "
for word in words:
    if len(word) > len(longest_word):
        longest_word = word
print(f"Das längste wort ist: {longest_word}")

print("Funktionen")
print("==========")

def sum_and_product(x,y): #x=1 , y=1 (prävention von scheiße)
    summe = x + y
    product = x * y
    return(summe,product)

print("Summe und Produkt: ",
      sum_and_product(3,4))

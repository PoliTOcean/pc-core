import json
import requests
import time
a=0
indirizzo='http://192.168.4.1'

while (a<100):
    print "richiedo..."
    r=requests.get(indirizzo)
    print r
    time.sleep(10)
    a+=1

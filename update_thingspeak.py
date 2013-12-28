import time
import serial
import httplib, urllib

thingSpeakKey = "00000000000000"  #PUT YOUR API KEY HERE
ser = serial.Serial('/dev/ttyAMA0', 57600)
fields = []

def updateThingSpeak(fields):
	print "Updating ThingSpeak ", time.strftime("%A, %B %d at %H:%M:%S")
	params = urllib.urlencode({'field1': fields[1], 'field2': fields[2], 'field3': fields[3], 'field4': fields[4], 'field5':fields[5],'field6':fields[6],'field7':fields[7],'field8':fields[8],'key':thingSpeakKey})
	headers = {"Content-type": "application/x-www-form-urlencoded","Accept":"text/plain"}
	conn = httplib.HTTPConnection("api.thingspeak.com:80")
	conn.request("POST", "/update", params, headers)
	response = conn.getresponse()
	print response.status, response.reason
	data = response.read()
	conn.close()
	print "response"
	print data

def main():
	while (1):
		line = ser.readline()
		if "data=" in line:
			#print line[:-2]         # strip \r\n
			fields = line[:-2].split(';')
			print line[:-2].split(';')
			wind = fields[1]
			gust = fields[2]
			rain = fields[3]
			temp = fields[4]
			light = fields[5]
               		humidity = fields[6]
                	vcc = fields[7]
                	ram = fields[8]
			updateThingSpeak(fields)

	#else:
		#print "invalid line"
		#print line
main()

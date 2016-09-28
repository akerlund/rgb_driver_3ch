import serial
import struct
import sys

def serial_func(mode):


	try:

		ser_com = serial.Serial('/dev/ttyUSB1', baudrate = 115200)

	except Exception as err:

		print ('Serial Exception')
		print (err)

	else:

		ser_com.flush( )

		#mode = 'tx0'

		if mode == 'rx0':
			try:

				while True:

					print ('Reading bytes.')
					read_data = ser_com.read(1)
					byte_rec = (struct.unpack('c',read_data)[0])
					print (byte_rec)
					print (type(byte_rec))

			except KeyboardInterrupt:

				print ('KeyboardInterrupt. Exiting.')


		elif mode == 'rx1':

			recieved = 0
			try:

				while True:

					read_data = ser_com.read(2)
					byte_rec = (struct.unpack('H',read_data)[0])
					print ('%i: %i.' % (recieved, byte_rec))
					
					recieved += 1
					if recieved == 16:
						recieved = 0

			except KeyboardInterrupt:

				print ('KeyboardInterrupt. Exiting.')


		elif mode == 'rx2':
			try:

				while True:

					print ('Reading ADC.')
					read_data = ser_com.read(2)
					byte_rec = (struct.unpack('H',read_data))
					for bytess in byte_rec:
						print(bytess)

			except KeyboardInterrupt:

				print ('KeyboardInterrupt. Exiting.')




		elif mode == 'tx0':
			try:

				while True:

					input_line = input ('Send data. > ')
					print (input_line)
					if input_line.isdigit( ):
						ser_com.write(struct.pack('h',int(input_line)))
						ser_com.flush()
						print ('Sent data: %s.' % input_line)

			except KeyboardInterrupt:

				print ('KeyboardInterrupt. Exiting.')



		ser_com.close( )

if __name__ == '__main__':

	print(sys.argv[1])
	serial_func(sys.argv[1])
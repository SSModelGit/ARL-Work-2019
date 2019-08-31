## @author Shashank Swaminathan
# @package WebToROS
# Layer connecting a local client to a server via non-blocking TCP sockets
#
# Uses standard system packages (sys, socket, json, types, argparse)
# As this is defined for Python2.7, it uses the special selectors2 package, in place of selectors. Download this package via pip.
# Defines the WebClientLayer class, a client-server connection handler
# Created by Shashank Swaminathan, June 2019, for work in ARL at TMSI, NUS

import sys
import socket
import json
try:
    import selectors
except ImportError:
    import selectors2 as selectors
import types
from argparse  import Namespace

## Connects to a server via TCP sockets and sends data as JSON-formatted strings
#
# Requires the server to be online before running
# Uses non-blocking sockets to handle the client-server requests
class WebClientLayer:
    ## Constructor method
    #
    # @param host Server host IP
    # @param port Server port
    # This does not start the connection to the server
    # This initializes three additional internal variables: max_mes_size, in_data, stored_data
    # max_mes_size is the maximum message size expected from the server (in bytes)
    # in_data is the buffer for data received from the server
    # stored_data is the data received from the server post-parsing
    def __init__(self, host, port):
        self.host = host
        self.port = port
        self.max_mes_size = 99999
        self.in_data = ""
        self.stored_data = ""

    ## Internal method for starting client-server connection
    #
    # @param message The message to send to the server
    def _start_connections(self, message):
        # Instantiate event handler
        self.sel = selectors.DefaultSelector()
        # Specify the server information
        server_addr = (self.host, self.port)
        # print("starting connection to", server_addr)
        # Bind to socket
        sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self.sock_ref = sock # JUST FOR USE IN FORCE CLOSING
        # Make the socket non-blocking
        sock.setblocking(False)
        # Connect to the server
        sock.connect_ex(server_addr)
        # Specify the events the socket will watch for
        events = selectors.EVENT_READ | selectors.EVENT_WRITE
        # Save the data relevant to the socket within a wrapper class
        data = Namespace(
            msg_total=self.max_mes_size,
            recv_total=0,
            message=message,
            outb="",
        )
        # Assign the socket, event information, and data to the event handler
        self.sel.register(sock, events, data=data)

    ## Internal method for servicing the connection to the server
    #
    # @param key Data associated with the connected socket received from the event scheduler
    # @param mask Value describes the type of event occuring
    # The server data is expected to be a JSON-formatted string.
    # It is also expected that before the '{' of the JSON string, there is a number prepended.
    # This number is the length of the incoming message from the server - this helps the layer realize when the message has been received.
    def _service_connection(self, key, mask):
        # Retrieve socket information from the event handler
        sock = key.fileobj
        data = key.data
        if mask & selectors.EVENT_READ: # If the event is to READ data from server
            recv_data = sock.recv(1024)  # Read data from socket
            if recv_data: # If data was received
                # print("received", recv_data)
                # Add received data to the internal buffer
                self.in_data += recv_data
                # Parse the data for the number detailing the length of the message
                msg_len_find = self.in_data.find("{")
                if msg_len_find is not -1:
                    # Once this number is found, update the expected message length field
                    data.msg_total = int(self.in_data[:msg_len_find])
                    self.in_data = self.in_data[msg_len_find:]
                data.recv_total += len(recv_data) # Update the amount of bytes received
            if not recv_data or data.recv_total == data.msg_total:
                # If there is no more data on the buffer, or if the amount of data received matches the expected amount
                # Close the client connection, and disconnect from the event handler
                # print("closing connection")
                self.sel.unregister(sock)
                sock.close()
        if mask & selectors.EVENT_WRITE: # If the event is to write data to the server
            if not data.outb and data.message:
                # If the outgoing buffer hasn't been initialized yet
                data.outb = data.message
                data.message = ""
            if data.outb:
                # After the outoging buffer has been prepared, send data to the server
                # print("sending", repr(data.outb), "to socket")
                sent = sock.send(data.outb)  # Writes data to socket
                data.outb = data.outb[sent:] # Clears out sent data from the outgoing buffer
        # self.expose = data # Meant to expose the internal buffer data for debugging

    ## Connects the device to the server via a client-server request routine
    #
    # @param message The message to send to the server.
    # The data sent to the server should be a properly formatted JSON string (formatted according to the server's expectations)
    # It will receive data from the server - this data is expected to be a JSON string as well
    # The data received from the server is stored within the stored_data field of the class.
    def connect_aws(self, message):
        # Starts connection to the server
        self._start_connections(message)
        try:
            while True:
                # Start event checking
                events = self.sel.select(timeout=1)
                if events: # Once events begin occuring
                    for key, mask in events:
                        # Service the events
                        self._service_connection(key, mask)
                # Check for a socket being monitored to continue. Otherwise, break.
                if not self.sel.get_map():
                    break
        except KeyboardInterrupt: # Handles KeyboardInterrupts
            print("caught keyboard interrupt, exiting")
        finally:
            # After everything has finished, store the data in in_data in stored_data
            # Clear out the internal buffer
            # End the event handler
            self.stored_data = self.in_data
            self.in_data = ""
            self.sel.close()

    # Deprecated socket closing routine.
    # def force_close(self):
    #     # Run in event of force close
    #     # This should be caught within connect_aws, but use this just as redundancy
    #     self.sel.unregister(self.sock_ref)
    #     self.sock_ref.close()
    #     self.sel.close()


# Deprecated class testing method.
# if __name__ == '__main__':
#     awsLayer = WebClientLayer('localhost', 9090)
#     mes = dict(linear=dict(x=90, y=360, z=0),
#                angular=dict(x=0, y=360, z=90))
#     # message = json.dumps(mes)
#     message = '{"m_id": 1722, "data": ["{\\"angular\\": {\\"y\\": 360, \\"x\\": 0, \\"z\\": 90}, \\"linear\\": {\\"y\\": 360, \\"x\\": 90, \\"z\\": 0}}"]}'
#     awsLayer.connect_aws(message)

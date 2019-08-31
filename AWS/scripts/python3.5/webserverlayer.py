## @author Shashank Swaminathan
# @package WebToROS
# Layer connecting the server to a client device running ROS
#
# Defines the WebServerLayer class, which sets up a server that uses non-blocking TCP sockets to communicate with a client running ROS.
# Expects specially formatted strings, following the JSON format standard.
# Created by Shashank Swaminathan, June 2019, for work in ARL at TMSI, NUS

import sys
import socket
import selectors
import types
import json

## Connects to a client device running ROS and sending JSON formatted requests
#
# Encodes message to clients using JSON - it prepends to the message the message length
# Uses non-blocking TCP sockets to connect to the client, to allow for multiple client connections
# Stores data from ROS in a custom-made helper object. This custom object acts as the server's API with the user.
# Occupies a thread on the computer. Other threads can send/read data to/from ROS via the helper object.
class WebServerLayer:
    ## Constructor method
    #
    # @param host IP of the server host
    # @param port Port number the server's socket will occupy
    def __init__(self, host, port):
        # Begin event handler
        self.sel = selectors.DefaultSelector()
        # Initialize and bind to socket
        lsock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        lsock.bind((host, port))
        # Start listening for incoming connections
        lsock.listen()
        print("listening on", (host, port))
        # Make the socket non-blocking
        lsock.setblocking(False)
        # Tell the event handler to watch the socket for 'READ' events (client connections)
        self.sel.register(lsock, selectors.EVENT_READ, data=None)

    ## Runs the main server code
    #
    # @param vault Object within which to store/read data to be stored/sent from/to ROS
    # This code will occupy a thread as it services multiple client connections
    # All data read from a client will be stored within the 'vault' object, and can be accessed using the device's unique ID
    # All data to be sent to a particular client should be stored within the 'vault', and should be stored using the device's unique ID
    def run(self, vault):
        try:
            while True:
                # Start checking events happening on the server socket
                events = self.sel.select(timeout=None)
                for key, mask in events: # When an event occurs
                    if key.data is None: # When the connection has not been accepted yet
                        self._accept_wrapper(key.fileobj) # Accepts/processes connection
                    else:
                        self._service_connection(key, mask, vault) # Services connection
        except KeyboardInterrupt: # Catch keyboard interrupts
            print("caught keyboard interrupt, exiting")
        finally:
            self.sel.close() # Close event handler after the server has been stopped

    ## Internal method: handles accepting client connections
    #
    # @param sock Server socket object
    def _accept_wrapper(self, sock):
        # Accepts client connection and stores client connection information
        conn, addr = sock.accept()
        print("accepted connection from", addr)
        # Make the socket connection nonblocking
        conn.setblocking(False)
        # Set the data to be associated with the socket
        data = types.SimpleNamespace(addr=addr, inb=b"", outb=b"")
        # Set the events the event handler will check for
        events = selectors.EVENT_READ | selectors.EVENT_WRITE
        # Associate the socket, desired events, and data in the event handler
        self.sel.register(conn, events, data=data)

    ## Internal method: services a given client connection
    #
    # @param key Data associated with the connected socket received from the event handler
    # @param mask Value describing the type of event occuring
    # @param vault Custom object storing data for server
    # The server follows basic nonblocking TCP protocol
    # It expects the data to be JSON formatted
    # The data received will be stored in the 'vault' object, under the client's device ID
    # Data to be sent to the client in response is read from the 'vault' object, using the client's device ID
    # Data to be stored in the vault is expected to be of the following form:
    # JSON(dict(m_id=DeviceID, data=list(JSON(ROS msg in dict form))))
    def _service_connection(self, key, mask, vault):
        # Retrieve socket information from the event handler
        sock = key.fileobj
        data = key.data
        if mask & selectors.EVENT_READ: # If the client is sending information
            recv_data = sock.recv(1024)  # Read data from the client
            if recv_data: # If there is data on the buffer
                data.inb += recv_data # Add data to internal buffer
                print("received", data.inb)
            else: # If there is no data on the buffer
                print("closing connection to", data.addr)
                # Close connection to the client socket
                self.sel.unregister(sock)
                sock.close()
        if mask & selectors.EVENT_WRITE: # If the server is sending data to the client
            if not data.outb and data.inb: # If the outgoing buffer has not been initialized
                # Store data from client within vault
                # It returns the client's unique device ID
                m_id = vault.store_ros_data(data.inb)
                # Re-initialize internal buffer to empty
                data.inb = ""
                # Retrieve data to send to the client from the vault using device ID
                # Prepend message length (in bytes) to data
                outgoing = self._i_add_len_header(vault.get_server_data(m_id))
                # Add to outgoing buffer
                data.outb = outgoing.encode("utf-8")
            if data.outb: # If outgoing buffer is initialized
                print("sending", repr(data.outb), "to", data.addr)
                sent = sock.send(data.outb)  # Writes data to socket
                data.outb = data.outb[sent:] # Clears out sent data from outgoing buffer

    ## Internal method: prepends message length to message
    #
    # @param message Message to prepend length to
    # It accounts for the additional length of the prepended length
    def _i_add_len_header(self, message):
        return str(len(str(len(message))) + len(message)) + message

## Custom helper class to store data related to client ROS devices
#
# Automatically parses stored data from clients
# Uses dictionaries to store data for unique clients
# Identifies clients via their device IDs
class ServerData:
    ## Constructor method
    #
    # Initializes an empty storage object
    def __init__(self):
        self.ros_data = dict()
        self.server_data = dict()
        self.blank_data = json.dumps(dict(topic="/radio_silence_serv",
                                          data=json.dumps(dict(data="yosh"))))

    ## Class method: stores ROS data from client
    #
    # @param data ROS data from the client
    # @return The unique device ID of the client (from where the data came from)
    # Expects data to be in the format specified within the server class
    def store_ros_data(self, data):
        parsed_data = json.loads(data.decode("utf-8"))
        m_id = parsed_data['m_id']
        self.ros_data[m_id] = list(map(json.loads, parsed_data['data']))
        return m_id

    ## Class method: get a dictionary of all stored ROS data
    #
    # @return A dictionary of all the stored ROS data
    # Dictionary's keys are the client devices' IDs
    def get_ros_data(self):
        return self.ros_data

    ## Class method: set the data to be sent to a specific client
    #
    # @param m_id Device ID of the client - specifies which key of the dictionary the data is stored under
    # @param data Data to be sent to the client specified by the device ID
    # The data is expected to be formatted as according to the client's expectations
    def set_server_data(self, m_id, data):
        self.server_data[m_id] = data

    ## Class method: get the stored data to be sent to a specific client
    #
    # @param m_id The device ID of the client whose data is to be retrieved
    # @return The data to be sent to the specified client
    def get_server_data(self, m_id):
        if m_id not in self.server_data.keys():
            self.server_data[m_id] = self.blank_data
        return self.server_data[m_id]


# The current implementation of the server
# Uses a constant to send to ROS (as only one thread is used)
if __name__ == '__main__':
    # Initialize objects - both storage and actual server object
    vault = ServerData()
    server = WebServerLayer('127.0.0.1', 9091)

    # Define message to send to ROS
    mes = dict(linear=dict(x=1, y=0, z=0),
               angular=dict(x=0, y=0, z=1))

    # Define message to send to the client, according to the client's formatting expectations
    message = json.dumps(dict(topic="/turtle1/cmd_vel",
                              data=json.dumps(mes)))

    # Add the data to the 'vault' storage object, under a specific device ID
    vault.set_server_data(1722, message)

    # Run the server, providing the storage object to be used (the 'vault' object)
    server.run(vault)

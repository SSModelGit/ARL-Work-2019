import sys
import socket
import selectors
import types
import json

class WebServerLayer:
    def __init__(self, host, port):
        self.sel = selectors.DefaultSelector()
        lsock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        lsock.bind((host, port))
        lsock.listen()
        print("listening on", (host, port))
        lsock.setblocking(False)
        self.sel.register(lsock, selectors.EVENT_READ, data=None)

    def run(self, vault):
        try:
            while True:
                events = self.sel.select(timeout=None)
                for key, mask in events:
                    if key.data is None:
                        self._accept_wrapper(key.fileobj)
                    else:
                        self._service_connection(key, mask, vault)
        except KeyboardInterrupt:
            print("caught keyboard interrupt, exiting")
        finally:
            self.sel.close()

    def _accept_wrapper(self, sock):
        conn, addr = sock.accept()  # Should be ready to read
        print("accepted connection from", addr)
        conn.setblocking(False)
        data = types.SimpleNamespace(addr=addr, inb=b"", outb=b"")
        events = selectors.EVENT_READ | selectors.EVENT_WRITE
        self.sel.register(conn, events, data=data)

    def _service_connection(self, key, mask, vault):
        sock = key.fileobj
        data = key.data
        if mask & selectors.EVENT_READ:
            recv_data = sock.recv(1024)  # Should be ready to read
            if recv_data:
                data.inb += recv_data
                print("received", data.inb)
            else:
                print("closing connection to", data.addr)
                self.sel.unregister(sock)
                sock.close()
        if mask & selectors.EVENT_WRITE:
            if not data.outb and data.inb:
                m_id = vault.store_ros_data(data.inb)
                data.inb = ""
                outgoing = self._i_add_len_header(vault.get_server_data(m_id))
                data.outb = outgoing.encode("utf-8")
            if data.outb:
                print("sending", repr(data.outb), "to", data.addr)
                sent = sock.send(data.outb)  # Should be ready to write
                data.outb = data.outb[sent:]

    def _i_add_len_header(self, message):
        return str(len(str(len(message))) + len(message)) + message

class ServerData:
    def __init__(self):
        self.ros_data = dict()
        self.server_data = dict()

    def store_ros_data(self, data):
        parsed_data = json.loads(data.decode("utf-8"))
        m_id = parsed_data['m_id']
        self.ros_data[m_id] = map(json.loads, parsed_data['data'])
        return m_id

    def get_ros_data(self):
        return self.ros_data

    def set_server_data(self, m_id, data):
        self.server_data[m_id] = data

    def get_server_data(self, m_id):
        return self.server_data[m_id]


if __name__ == '__main__':
    vault = ServerData()
    server = WebServerLayer('localhost', 9091)

    mes = dict(linear= dict(x=0,y=100,z=0),
               angular=dict(x=0,y=180,z=0))

    vault.set_server_data(1722, json.dumps(mes))

    server.run(vault)

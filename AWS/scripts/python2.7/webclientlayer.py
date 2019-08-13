import sys
import socket
import json
try:
    import selectors
except ImportError:
    import selectors2 as selectors
import types
from argparse  import Namespace

class WebClientLayer:
    def __init__(self, host, port):
        self.host = host
        self.port = port
        self.max_mes_size = 99999
        self.in_data = ""
        self.stored_data = ""

    def _start_connections(self, message):
        self.sel = selectors.DefaultSelector()
        server_addr = (self.host, self.port)
        # print("starting connection to", server_addr)
        sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self.sock_ref = sock # JUST FOR USE IN FORCE CLOSING
        sock.setblocking(False)
        sock.connect_ex(server_addr)
        events = selectors.EVENT_READ | selectors.EVENT_WRITE
        data = Namespace(
            msg_total=self.max_mes_size,
            recv_total=0,
            message=message,
            outb="",
        )
        self.sel.register(sock, events, data=data)

    def _service_connection(self, key, mask):
        sock = key.fileobj
        data = key.data
        if mask & selectors.EVENT_READ:
            recv_data = sock.recv(1024)  # Should be ready to read
            if recv_data:
                # print("received", recv_data)
                self.in_data += recv_data
                msg_len_find = self.in_data.find("{")
                if msg_len_find is not -1:
                    data.msg_total = int(self.in_data[:msg_len_find])
                    self.in_data = self.in_data[msg_len_find:]
                data.recv_total += len(recv_data)
            if not recv_data or data.recv_total == data.msg_total:
                # print("closing connection")
                self.sel.unregister(sock)
                sock.close()
        if mask & selectors.EVENT_WRITE:
            if not data.outb and data.message:
                data.outb = data.message
                data.message = ""
            if data.outb:
                # print("sending", repr(data.outb), "to socket")
                sent = sock.send(data.outb)  # Should be ready to write
                data.outb = data.outb[sent:]
        self.expose = data

    def connect_aws(self, message):
        self._start_connections(message)
        try:
            while True:
                events = self.sel.select(timeout=1)
                if events:
                    for key, mask in events:
                        self._service_connection(key, mask)
                # Check for a socket being monitored to continue.
                if not self.sel.get_map():
                    break
        except KeyboardInterrupt:
            print("caught keyboard interrupt, exiting")
        finally:
            self.stored_data = self.in_data
            self.in_data = ""
            self.sel.close()

    # def force_close(self):
    #     # Run in event of force close
    #     # This should be caught within connect_aws, but use this just as redundancy
    #     self.sel.unregister(self.sock_ref)
    #     self.sock_ref.close()
    #     self.sel.close()


if __name__ == '__main__':
    awsLayer = WebClientLayer('localhost', 9090)
    mes = dict(linear=dict(x=90, y=360, z=0),
               angular=dict(x=0, y=360, z=90))
    # message = json.dumps(mes)
    message = '{"m_id": 1722, "data": ["{\\"angular\\": {\\"y\\": 360, \\"x\\": 0, \\"z\\": 90}, \\"linear\\": {\\"y\\": 360, \\"x\\": 90, \\"z\\": 0}}"]}'
    awsLayer.connect_aws(message)

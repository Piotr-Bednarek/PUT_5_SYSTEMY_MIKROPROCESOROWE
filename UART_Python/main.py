import serial
import serial.tools.list_ports
import tkinter as tk
from tkinter import ttk, scrolledtext, messagebox
import threading
import time
from matplotlib.backends.backend_tkagg import FigureCanvasTkAgg
from matplotlib.figure import Figure
import matplotlib.pyplot as plt

class UARTApp:
    def __init__(self, root):
        self.root = root
        self.root.title("UART Komunikator")
        self.root.state('zoomed')
        self.serial = None
        self.running = False
        self.buffer = ""
        
        config_frame = ttk.LabelFrame(root, text="Konfiguracja", padding=10)
        config_frame.pack(fill="x", padx=10, pady=10)
        
        # porty szeregowe
        ttk.Label(config_frame, text="Port:").grid(row=0, column=0, sticky="w")
        self.port = tk.StringVar()
        self.port_combo = ttk.Combobox(config_frame, textvariable=self.port, width=60)
        self.port_combo.grid(row=0, column=1, sticky="w", padx=5)
        self.refresh_ports()
        
        ttk.Button(config_frame, text="Odśwież", command=self.refresh_ports).grid(row=0, column=2, padx=5)
        
        # button polacz
        self.connect_button = ttk.Button(config_frame, text="Połącz", command=self.toggle_connection)
        self.connect_button.grid(row=0, column=3, padx=5)
        
        # baudrate
        ttk.Label(config_frame, text="Baudrate:").grid(row=1, column=0, sticky="w", pady=5)
        self.baudrate = tk.StringVar(value="9600")
        self.baudrate_combo = ttk.Combobox(config_frame, textvariable=self.baudrate, 
                                        values=["9600", "115200", "19200", "38400", "57600"], width=15)
        self.baudrate_combo.grid(row=1, column=1, sticky="w", padx=5)
        
        # databitsy
        ttk.Label(config_frame, text="Data bits:").grid(row=2, column=0, sticky="w", pady=5)
        self.databits = tk.StringVar(value="8")
        self.databits_combo = ttk.Combobox(config_frame, textvariable=self.databits, 
                                           values=["5", "6", "7", "8"], width=15)
        self.databits_combo.grid(row=2, column=1, sticky="w", padx=5)
        
        # parirt
        ttk.Label(config_frame, text="Parity:").grid(row=3, column=0, sticky="w", pady=5)
        self.parity = tk.StringVar(value="None")
        self.parity_combo = ttk.Combobox(config_frame, textvariable=self.parity, 
                                         values=["None", "Even", "Odd", "Mark", "Space"], width=15)
        self.parity_combo.grid(row=3, column=1, sticky="w", padx=5)
        
        # stopbitsy
        ttk.Label(config_frame, text="Stop bits:").grid(row=4, column=0, sticky="w", pady=5)
        self.stopbits = tk.StringVar(value="1")
        self.stopbits_combo = ttk.Combobox(config_frame, textvariable=self.stopbits, 
                                          values=["1", "1.5", "2"], width=15)
        self.stopbits_combo.grid(row=4, column=1, sticky="w", padx=5)
        
        # status polaczenia z serialem
        self.status_label = ttk.Label(config_frame, text="Status: Rozłączony", foreground="red")
        self.status_label.grid(row=0, column=4, columnspan=4, sticky="w", pady=5)
        
        # frame transmit
        transmit_frame = ttk.LabelFrame(root, text="Wysyłanie", padding=10)
        transmit_frame.pack(fill="x", padx=10, pady=5)
        
        ttk.Label(transmit_frame, text="Dane:").grid(row=0, column=0, sticky="w")
        self.transmit_entry = ttk.Entry(transmit_frame, width=50)
        self.transmit_entry.grid(row=0, column=1, padx=5)
        
        ttk.Button(transmit_frame, text="Wyślij", command=self.transmit_data).grid(row=0, column=2, padx=5)
        
        # frame dla receiva i wykresu
        main_frame = ttk.Frame(root)
        main_frame.pack(fill="both", expand=True, padx=10, pady=5)
        
        # frame dla receive
        receive_frame = ttk.LabelFrame(main_frame, text="Odebrane dane", padding=10)
        receive_frame.pack(side="left", fill="both", expand=True, padx=(0, 5))
        
        self.receive_text = scrolledtext.ScrolledText(receive_frame, height=12, width=20, state="disabled")
        self.receive_text.pack(fill="both", expand=True)
        
        ttk.Button(receive_frame, text="Wyczyść", command=self.clear_receive).pack(side="left", pady=5)
        
        # frame wykres
        chart_frame = ttk.LabelFrame(main_frame, text="Wykres", padding=10)
        chart_frame.pack(side="right", fill="both", expand=True, padx=(5, 0))
        
        # frame dla ustawien wykresu
        axis_frame = ttk.Frame(chart_frame)
        axis_frame.pack(fill="x", pady=(0, 10))
        
        ttk.Label(axis_frame, text="Oś X:").grid(row=0, column=0, sticky="w", padx=5)
        self.x_axis_label = tk.StringVar(value="X")
        ttk.Entry(axis_frame, textvariable=self.x_axis_label, width=15).grid(row=0, column=1, padx=5)
        
        ttk.Label(axis_frame, text="Oś Y:").grid(row=0, column=2, sticky="w", padx=5)
        self.y_axis_label = tk.StringVar(value="Y")
        ttk.Entry(axis_frame, textvariable=self.y_axis_label, width=15).grid(row=0, column=3, padx=5)

        ttk.Label(axis_frame, text="Tytuł wykresu:").grid(row=0, column=4, sticky="w", padx=5)
        self.title_label = tk.StringVar(value="Dane z UART")
        ttk.Entry(axis_frame, textvariable=self.title_label, width=15).grid(row=0, column=5, padx=5)
        
        ttk.Button(axis_frame, text="Wyczyść", command=self.clear_chart).grid(row=0, column=7, padx=5)
        
        self.chart_enabled = tk.BooleanVar(value=True)
        self.chart_button = ttk.Button(axis_frame, text="Wykres: ON", command=self.toggle_chart)
        self.chart_button.grid(row=0, column=6, padx=5)
        
        # wykresik
        self.fig = Figure(figsize=(5, 4), dpi=100)
        self.ax = self.fig.add_subplot(111)
        self.ax.set_title("Dane z UART")
        self.ax.set_xlabel("X")
        self.ax.set_ylabel("Y")

        # to nie wiem co to jest 
        self.canvas = FigureCanvasTkAgg(self.fig, master=chart_frame)
        self.canvas.get_tk_widget().pack(fill="both", expand=True)

        self.data_x = []
        self.data_y = []
        self.point_count = 0
        
        # odbieranie danych w tle
        self.read_thread = threading.Thread(target=self.read_loop, daemon=True)
        self.read_thread.start()
    
    def refresh_ports(self):
        ports = []
        stm_port = None
        for port in serial.tools.list_ports.comports():
            device = port.device
            description = port.description
            # print(device, description)
            if "STMicroelectronics STLink Virtual COM Port" in description:
                stm_port = f"{device} - {description}"
            if (device and description):
                ports.append(f"{device} - {description}")
        
        
        self.port_combo['values'] = ports
        if ports:
            self.port_combo.current(0)

        if stm_port:
            self.port_combo.set(stm_port)
    
    def toggle_connection(self):
        if self.serial:
            self.disconnect()
        else:
            self.connect()
    
    def connect(self):
        try:
            port_full = self.port.get()
            port_value = port_full.split(" - ")[0]
            baudrate_value = int(self.baudrate.get())
            databits_value = int(self.databits.get())
            stopbits_value = self.stopbits.get()
            parity_value = self.parity.get()
            
            parity_map = {
                "None": serial.PARITY_NONE,
                "Even": serial.PARITY_EVEN,
                "Odd": serial.PARITY_ODD,
                "Mark": serial.PARITY_MARK,
                "Space": serial.PARITY_SPACE
            }
            parity = parity_map[parity_value]
            
            stopbits_map = {
                "1": serial.STOPBITS_ONE,
                "1.5": serial.STOPBITS_ONE_POINT_FIVE,
                "2": serial.STOPBITS_TWO
            }
            stopbits = stopbits_map[stopbits_value]
            
            self.serial = serial.Serial(port_value, baudrate_value, timeout=0.1, 
                                     bytesize=databits_value, parity=parity, stopbits=stopbits)
            self.status_label.config(text=f"Status: Połączony ({port_full})", foreground="green")
            self.connect_button.config(text="Rozłącz")
            self.port_combo.config(state="disabled")
            self.baudrate_combo.config(state="disabled")
            self.databits_combo.config(state="disabled")
            self.parity_combo.config(state="disabled")
            self.stopbits_combo.config(state="disabled")
            self.running = True
            self.log_message(f"[SYSTEM] Połączono z {port_full} @ {baudrate_value} bps ({databits_value} bits, {parity_value}, {stopbits_value} stop)\n")
        except Exception as e:
            messagebox.showerror(f"Nie mogę się połączyć:\n{e}")
    
    def disconnect(self):
        if self.serial:
            self.serial.close()
            self.serial = None
        self.status_label.config(text="Status: Rozłączony", foreground="red")
        self.connect_button.config(text="Połącz")
        self.port_combo.config(state="normal")
        self.baudrate_combo.config(state="normal")
        self.databits_combo.config(state="normal")
        self.parity_combo.config(state="normal")
        self.stopbits_combo.config(state="normal")
        self.running = False
        self.log_message("[SYSTEM] Rozłączono\n")
    
    def transmit_data(self):
        if not self.serial:
            return
        
        data = self.transmit_entry.get()
        if data:
            try:
                self.serial.write(data.encode() + b'\n')
                self.log_message(f"[TX] {data}\n")
                self.transmit_entry.delete(0, tk.END)
            except Exception as e:
                messagebox.showerror(f"Błąd wysyłania:\n{e}")
    
    def read_loop(self):
        while True:
            try:
                if self.serial and self.serial.is_open and self.running:
                    if self.serial.in_waiting > 0:
                        data = self.serial.read(self.serial.in_waiting).decode(errors='ignore')
                        self.buffer += data
                        
                        while '\n' in self.buffer:
                            line, self.buffer = self.buffer.split('\n', 1)
                            self.log_message(f"{line}\n")
                            self.update_chart(line)  # Dodaj do wykresu
            except (OSError, AttributeError, serial.SerialException):
                pass
            time.sleep(0.1)
    
    def log_message(self, message):
        self.receive_text.config(state="normal")
        self.receive_text.insert(tk.END, message)
        self.receive_text.see(tk.END)
        self.receive_text.config(state="disabled")
    
    def clear_receive(self):
        self.receive_text.config(state="normal")
        self.receive_text.delete(1.0, tk.END)
        self.receive_text.config(state="disabled")
    
    def toggle_chart(self):
        """Włącz/wyłącz wykres"""
        self.chart_enabled.set(not self.chart_enabled.get())
        if self.chart_enabled.get():
            self.chart_button.config(text="Wykres: ON")
        else:
            self.chart_button.config(text="Wykres: OFF")
    
    def update_chart(self, value):
        if not self.chart_enabled.get():
            return
        
        try:
            if ',' in value:
                parts = value.strip().split(',')
                if len(parts) >= 2:
                    x_val = float(parts[0])
                    y_val = float(parts[1])
                else:
                    return
            self.data_x.append(x_val)
            self.data_y.append(y_val)
            self.point_count += 1
            
            # Ogranicz ilość punktów na wykresie (np. 100)
            if len(self.data_x) > 100:
                self.data_x.pop(0)
                self.data_y.pop(0)
            
            # Odśwież wykres
            self.ax.clear()
            self.ax.plot(self.data_x, self.data_y, 'b-')
            self.ax.set_title(self.title_label.get())
            self.ax.set_xlabel(self.x_axis_label.get())
            self.ax.set_ylabel(self.y_axis_label.get())
            self.canvas.draw()
        except (ValueError, IndexError):
            # Jeśli wartość nie jest liczbą, ignoruj
            pass

    def clear_chart(self):
        self.data_x = []
        self.data_y = []
        self.point_count = 0
        self.ax.clear()
        self.canvas.draw()

        
    

if __name__ == "__main__":
    root = tk.Tk()
    app = UARTApp(root)
    root.mainloop()

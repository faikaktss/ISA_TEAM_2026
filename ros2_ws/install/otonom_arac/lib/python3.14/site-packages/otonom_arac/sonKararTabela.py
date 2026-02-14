from info import Info
from arduino import Teensy,Arduino
import cv2
import time

class komutcu2:
    def __init__(self,inf,teensy,arduino):
            self.inf = inf
            self.durakPixel = 0
            self.teensy = teensy # Todo: Teens (motor Kontrolü)
            self.arduino = arduino # Todo: Sensör okuma
    def komut(self):
        while(1):
            engelflag = 0 
            kronometre = 0
            tab = self.inf.get_tabela()
            # Todo: Duran engel
            if self.inf.get_engel() == 1:
                while(kronometre<1.75):
                    print("engelden kaciyom")
                    time.sleep(0.1)
                    kronometre+=0.1
                    self.teensy.setValue("-25") # Todo: sol dön
                kronometre = 0 
                while(kronometre<3.5):
                    time.sleep(0.1)
                    kronometre+=0.1
                    self.teensy.setValue("25") # Todo: sağa dön
            elif (self.inf.get_engel() == 2): # Todo: hareketli engel
                        while (self.inf.get_engel() != 0):
                            print("Hareketli engel dur!")
                            self.teensy.setValue("100")
                            time.sleep(0.1) 
                        print("Engel kalktı, devam ediliyor.")
            else:
                self.durakPixel = self.inf.durakPixel
                print(f"pixel sayısı :{self.durakPixel}")
                if self.inf.get_durak_engel() == 1:
                     engelflag = 1
                     # Todo: piksel büyükse yakınlaştığını anlıyor
                if(self.durakPixel>990 and engelflag== 0):
                    if(self.inf.get_angle() is not None):
                        self.teensy.setValue(str(self.inf.get_angle())) # Todo: şerit takibi ile yaklaş
                    time.sleep(0.9)
                    print("durak bekleniyo" + str(self.durakPixel))
                    if self.inf.get_durak_engel() == 1:
                         engelflag = 1

                    if engelflag== 0:
                        while(kronometre<3.5):
                            time.sleep(0.1)
                            kronometre+=0.1
                            self.teensy.setValue("19") 
                        kronometre = 0
                        while(kronometre<4):
                            kronometre+=0.1
                            time.sleep(0.1)
                            if(self.inf.get_angle() is not None):
                                self.teensy.setValue(str(self.inf.get_angle()))
                        kronometre = 0 
                        while(kronometre<5):
                            kronometre+=0.1
                            time.sleep(0.1)
                            self.teensy.setValue("100")
                        kronometre = 0 
                        while(kronometre<4):
                            kronometre+=0.1
                            time.sleep(0.1)
                            self.teensy.setValue("-27")
                    else:
                         aci = self.inf.get_angle()
                         if(aci is not None):
                         
                            self.teensy.setValue(str(int(aci)))
                            time.sleep(0.1)
                else:
                    aci = self.inf.get_angle()
                    if(aci is not None):
                         
                        self.teensy.setValue(str(int(aci)))
                        time.sleep(0.1)
            time.sleep(0.1) 


class komutcu:
    def __init__(self,inf,teensy,arduino):
            self.inf = inf # Todo: Robot verileri
            self.durakPixel = 0
            self.teensy = teensy # Todo: Teens (motor Kontrolü)
            self.arduino = arduino # Todo: Sensör okuma
            self.tabela_gecmisi = []
            self.yeni_tab = None

    # Todo: Tensy'nin buradaki kullanımı Direksiyon açısı ve hız kontorolüdür
    def komut(self):
            while(1):
                    kronometre = 0
                    tab = self.inf.get_tabela()
                    
                    # Todo : Hareketkli engel kontrolü
                    if self.inf.get_engel() == 2:
                        kronometre=0 
                        while (kronometre<5 and self.inf.get_engel() != 0 ):
                            kronometre+=0.1
                            print("Hareketli engel dur!")
                            self.teensy.setValue("100") # Todo: 100 komutu ile durur
                            time.sleep(0.2) 

                        print("Engel kalktı, devam ediliyor.")
                        self.teensy.setValue(str(self.inf.get_angle()))
                    
                    # Todo: Sabit engel kontrolü
                    elif self.inf.get_engel() == 1:
                        while(kronometre<3):
                            print("engelden kaciyom")
                            time.sleep(0.1)
                            kronometre+=0.1
                            self.teensy.setValue("-30")
                        kronometre = 0 
                        while(kronometre<3):
                            time.sleep(0.1)
                            kronometre+=0.1
                            self.teensy.setValue("30")
                        kronometre = 0 
                        while(kronometre<6.7):
                            time.sleep(0.1)
                            kronometre+=0.1
                            self.teensy.setValue("25")


                

                    # Todo: Yolo ile alınan tabela komutları
                    else:
                        #tabela geçmişi
                        if tab is not None:
                            self.tabela_gecmisi.append(tab)
                            if len(self.tabela_gecmisi) > 5:
                                self.tabela_gecmisi.pop(0)
                        #eğer son 5 tabela aynıysa
                        if len(self.tabela_gecmisi) == 5 and all(x is not None and x == self.tabela_gecmisi[0] for x in self.tabela_gecmisi) :
                            self.yeni_tab = self.tabela_gecmisi[0]
                            print(f"{self.yeni_tab} komutu başlatılıyor (5 kez algılandı).")
                        
                        kronometre = 0
                        if self.yeni_tab =="durak":
                            while(self.durakPixel<1200):
                                self.durakPixel = self.inf.durakPixel
                                if(self.inf.get_angle() is not None):
                                    self.teensy.setValue(str(self.inf.get_angle()))
                                time.sleep(0.1)
                                print("durak bekleniyo" + str(self.durakPixel))
                            while(kronometre<4.4):
                                time.sleep(0.1)
                                kronometre+=0.1
                                self.teensy.setValue("20") 
                            kronometre = 0
                            while(kronometre<4):
                                kronometre+=0.1
                                time.sleep(0.1)
                                if(self.inf.get_angle() is not None):
                                    self.teensy.setValue(str(self.inf.get_angle()))
                            kronometre = 0 
                            while(kronometre<3.5):
                                kronometre+=0.1
                                time.sleep(0.2)
                                self.teensy.setValue("100")
                            kronometre = 0 
                            while(kronometre<4.6):
                                kronometre+=0.1
                                time.sleep(0.1)
                                self.teensy.setValue("-23")
                            self.yeni_tab = None
                            self.tabela_gecmisi.clear()
                        #uzaklik kontrolü eklencek
                        elif tab =="park_yasakiki":
                            kronometre = 0
                            while(kronometre<10 and (self.inf.get_tabela() !="sol_yasak")):
                                kronometre+=0.1
                                time.sleep(0.1)
                                print("bekle")
                                self.teensy.setValue("100") # Todo: Araba durur
                            self.yeni_tab = None
                            self.tabela_gecmisi.clear()
                        
                        elif self.yeni_tab == "dur":
                            kronometre = 0
                            while(kronometre <8):
                                kronometre+=0.1
                                time.sleep(0.2)
                                print("Dur!")
                                self.teensy.setValue("100") # Todo: 100 komutu ile durur
                            self.yeni_tab = None
                            self.tabela_gecmisi.clear()
                        
                        elif self.yeni_tab == "girilmez":
                            if self.inf.get_solRedCount() >= 10:
                                print("Arac sola dönmeye hazir.")
                                kronometre=0
                                while (kronometre<5):
                                    print("BBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBB")
                                    kronometre+=0.1
                                    time.sleep(0.1)
                                    print("Arac sola dönüyor...")
                                    self.teensy.setValue("-20")
                                kronometre = 0
                                while(kronometre<4):
                                    print("AAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAA")
                                    kronometre+=0.1
                                    time.sleep(0.1)
                                    if(self.inf.get_angle() is not None):
                                        self.teensy.setValue(str(self.inf.get_angle()))
                                self.yeni_tab = None
                                self.tabela_gecmisi.clear()
                        
                        elif self.yeni_tab == "sag":
                            if self.inf.get_sagRedCount() >= 0:
                                print("Arac saga dönmeye hazir.")
                                kronometre=0
                                while (kronometre<5):
                                    print("BBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBB")
                                    kronometre+=0.1
                                    time.sleep(0.1)
                                    print("Arac saga dönüyor...")
                                    self.teensy.setValue("20")
                                kronometre = 0
                                while(kronometre<4):
                                    print("AAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAA")
                                    kronometre+=0.1
                                    time.sleep(0.1)
                                    if(self.inf.get_angle() is not None):
                                        self.teensy.setValue(str(self.inf.get_angle()))
                                self.yeni_tab = None
                                self.tabela_gecmisi.clear()

                        else:
                            
                            print("bisey yok aci atiyom")
                            if(self.inf.get_angle() is not None):
                                self.teensy.setValue(str(self.inf.get_angle()))
                                time.sleep(0.1)
                        time.sleep(0.01)
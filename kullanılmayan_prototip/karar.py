from komut import YonluTab, tabela, DurTab, DevamTab, DurakTab
from time import sleep
from info import Info
from arduino import Arduino, Teensy
import utils
from camera import cam

tabelaEsles = {
    "kirmizi": (DurTab, 0),
    "yesil": (DevamTab, 2),
    "park": (tabela, None),
    "park_engelli": (tabela, None),
    "sol": (YonluTab, -1),
    "sag": (YonluTab, 1),
    "sag_yasak": (DevamTab, 2),
    "sol_yasak": (DevamTab, 2),
    "dur": (DurTab, 0),
    "durak": (DurakTab, 0),
    "girilmez": (tabela, None),
    "sol": (YonluTab, -1),
    "kavsak": (YonluTab, -1),
    "sagdan_devam": (YonluTab, 1),
    "soldan_devam": (YonluTab, -1),
    "park_yasakiki": (tabela, None),
    "yaya": (DurTab, 0),
    "tunel": (tabela, None),
    "ileri": (DevamTab, 2),
    "cift_yon": (tabela, None)
}

def komut(tabela_adi, arduino=None, teensy=None, distance=0):
    print("komuttayım")
    if tabela_adi not in tabelaEsles:
        print(f"{tabela_adi} için eşleşme bulunamadı.")
        return None

    sinif, yon = tabelaEsles[tabela_adi]

    if isinstance(sinif, type) and issubclass(sinif, YonluTab):
        print("ifffff")
        return sinif(tabela_adi, 0, distance, 0, yon)
    elif isinstance(sinif, type) and issubclass(sinif, (DurTab, DevamTab, DurakTab)):
        return sinif(tabela_adi, 0, distance, 0,yon)
    else:
        return sinif(tabela_adi, 0, distance, 0)


class EngelKomut:
    def __init__(self, info: Info, teensy: Teensy, engel_tipi=1):
        self.bitti = False
        self.engel_tipi = engel_tipi  # 1: hareketsiz, 2: hareketli
        self.info = info
        self.teensy = teensy

    def islem(self):
        if not self.bitti:
            if self.engel_tipi == 2:
                print("HAREKETLİ ENGELLLLL VAAAR")
                for _ in range(100):
                    self.teensy.setValue("0")
                    print("açı gönderiyorum hareketli engel")
            else:
                # lane = self.info.get_lane() if self.info else "bilinmiyor"
                # if lane == 1:
                print("ENGEEEELLLL VAAAR - SAĞ ŞERİTTEYİZ")
                for _ in range(100):
                    self.teensy.setValue("-35")
                    print("açı gönderiyorum")

                # elif lane == -1:
                #     print("ENGEEEELLLL VAAAR - SOL ŞERİTTEYİZ")
                # else:
                #     print("ENGEEEELLLL VAAAR - ŞERİT BİLGİSİ YOK")
            self.bitti = True

    def bitti_mi(self):
        return self.bitti


class Command:
    def __init__(self, info: Info, teensy: Teensy, arduino: Arduino, camera: cam):
        self.info = info
        self.teensy = teensy
        self.arduino = arduino
        self.utils = utils
        self.state = 0
        self.distance = 0
        self.tabela = []
        self.komut_tabela = []
        self.camera = camera
        self.engel_komutu_aktif = False
        self.engel_komut = EngelKomut(info=self.info, teensy=self.teensy)

    def engel_kontrol(self):
        engel_durumu = self.info.get_engel()

        if engel_durumu > 0 and not self.engel_komutu_aktif:
            print("ENGEL TESPİT EDİLDİ, kaçınma başlatılıyor...")
            self.engel_komut = EngelKomut(engel_tipi=engel_durumu, info=self.info, teensy=self.teensy)
            self.engel_komutu_aktif = True

        if self.engel_komutu_aktif:
            self.engel_komut.islem()
            if self.engel_komut.bitti_mi():
                print("Kaçınma tamamlandı.")
                self.engel_komutu_aktif = False

    def karar(self, tabela_list, sol_red_count, sag_red_count):
        son_bes = tabela_list[-5:]
        print("karardayim")

        if len(son_bes) == 5 and all(x is not None and x == son_bes[0] for x in son_bes) and self.state == 0:
            yeni_tabela = komut(son_bes[0])
            if yeni_tabela:
                self.tabela.append(yeni_tabela)
                self.state = 1
                print(f"{yeni_tabela.isim} komutu eklendi.")

        if self.tabela and (not self.komut_tabela):
            aktif = self.tabela[-1]
            if isinstance(aktif, YonluTab):
                self.yonlu_sign_control(aktif, sol_red_count, sag_red_count)
            elif isinstance(aktif, DurTab):
                pass
            elif isinstance(aktif, DevamTab):
                pass
            elif isinstance(aktif, DurakTab):
                self.check_durak_tab(aktif, sol_red_count, sag_red_count)
            elif isinstance(aktif, tabela):
                pass

        if self.komut_tabela:
            aktif = self.komut_tabela[-1]
            if isinstance(aktif, tabela):
                aktif.islem(self.tabela, self.komut_tabela, self.arduino, self.teensy, self.info)
                self.state = 0
            else:
                aktif.islem(self.komut_tabela, self.arduino, self.teensy, self.info)

        if not self.komut_tabela and not self.tabela:
            self.arduino.setValue("2000")
            self.teensy.setValue(str(self.info.get_angle()))

    def yonlu_sign_control(self, yonlu: YonluTab, sol_red_count, sag_red_count):
        if yonlu.kosul(sol_red_count, sag_red_count):
            print(f"{yonlu.yon} yönlü tabela için şart sağlandı. Dönüş yapılıyor.")
            self.komut_tabela.append(yonlu)
            self.state = 0
        else:
            print(f"{yonlu.yon} yönlü tabela için şart sağlanamadı. Bekleniyor.")

    def check_durak_tab(self, durak_t: DurakTab, sol_red_count, sag_red_count):
        durak_var, bird_frame = durak_t.kosul(self.camera.img, sol_red_count, sag_red_count)

        if durak_var:
            print(f"{durak_t.isim} tabelası için şart sağlandı. Durak işlemi başlatılıyor.")
            self.info.set_description(f"{durak_t.isim} tabelası algılandı. Araç durağa giriyor.")
            bird_frame = durak_t.islem(self.camera.img, self.komut_tabela, self.arduino, self.teensy, self.info)
            self.state = 0
        else:
            print(f"{durak_t.isim} tabelası için şart sağlanamadı. Bekleniyor.")

        

   
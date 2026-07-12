# UAV-GCS "Gerçek Dünya" Kamera Görünümü — CesiumJS Tabanlı Bileşen Talebi

## BAĞLAM VE SORUN
Savunma sanayi tarzı bir UAV Ground Control Station dashboard'unun parçası.
Şu anda "kamera" panelimiz bir fizik motorundaki (Gazebo) JENERİK/kurgusal bir
arazi üzerinde render ediliyor — dashboard'daki harita panelindeki GERÇEK
konum (uydu görüntüsü, gerçek şehir/arazi) ile bu kamera görüntüsündeki arazi
BİRBİRİYLE UYUŞMUYOR (biri gerçek Eskişehir, diğeri rastgele bir simülasyon
tepeliği). Bunu çözmek istiyorum: kamera görünümünü CesiumJS ile GERÇEK
dünya verisine (gerçek arazi yüksekliği/DEM + gerçek uydu görüntüsü) bağlayıp,
"drone'un GERÇEK koordinatta, GERÇEK arazi üzerinde ne görürdü" görünümünü
üretmek istiyorum.

Bu bileşen mevcut Gazebo tabanlı video panelinin YERİNE geçecek. Panelin
üstüne bindirilen ayrı bir OSD (crosshair, pitch ladder, MGRS/LAT/LON, LINK
durumu vb.) katmanı zaten var ve ayrı bir bileşen — SEN SADECE ALTTAKİ 3D
GÖRÜNÜMÜ üretiyorsun, OSD'yi tekrar üretme.

## TEKNİK ÇERÇEVE
- CesiumJS'i CDN üzerinden yükle (`<helmet>` içine
  `https://cesium.com/downloads/cesiumjs/releases/<son-surum>/Build/Cesium/Cesium.js`
  + ilgili `Widgets/widgets.css`), önceki panelde Leaflet'i nasıl CDN'den
  yüklediysek aynı desen.
- Çıktı formatı önceki panellerle birebir aynı: kendi kendine yeten `.dc.html`
  + `support.js` (dc-runtime), `data-props` ile parametrik.
- Bileşen kendi `<div>` konteynerini dolduran tam boy bir Cesium `Viewer`
  (veya performans için sade bir `Scene`/`Camera` kurulumu, tam Viewer widget
  chrome'una — arama kutusu, ölçüm aracı, base layer picker vb. — GEREK YOK,
  hepsini kapat, sadece render alanı kalsın).
- Cesium ion erişimi bir API TOKEN gerektiriyor (ücretsiz hesap) — bunu sabit
  kodlama, `cesium_ion_token` adında bir string prop olarak bırak (boş
  string default, ben kendi token'ımı gireceğim).

## KAMERA DAVRANIŞI (EN ÖNEMLİ KISIM)
İki mod olsun, prop ile seçilebilir (`camera_mode`: "pov" | "chase"):

1. **"pov" (drone'un gözünden)** — Cesium kamerasını doğrudan drone'un
   konumuna (`lat`, `lon`, `alt_m` prop'ları — WGS84 derece/metre) yerleştir.
   Kameranın bakış yönünü (`heading`) `yaw_deg` prop'undan, `pitch`'i
   `pitch_deg` prop'undan al, `roll`'u da varsa uygula (Cesium
   `camera.setView({destination, orientation: {heading, pitch, roll}})`
   API'si tam bunun için var). Yani kart/drone fiziksel olarak eğildiğinde
   gerçek arazi üzerinde gerçek zamanlı POV dönmeli.

2. **"chase" (üçüncü şahıs)** — kameranın birkaç metre arkasında/üstünde
   sabit bir ofsetle drone'u takip ettiği, drone'un konum/yön'üne göre
   otomatik hesaplanan bir görünüm (drone'un kendisini görsel olarak temsil
   eden basit bir 3D model/marker de sahnede dursun — karmaşık bir model
   gerekmiyor, basit bir koni/ok şekli yeterli, `Cesium.Entity` ile).

Varsayılan `camera_mode="pov"` olsun (asıl istediğimiz bu — gerçek "drone'un
gördüğü" hissi).

## ARAZİ / GÖRÜNTÜ KATMANLARI
- `Cesium.createWorldTerrainAsync()` (veya güncel API neyse) ile GERÇEK küresel
  arazi yükseklik verisi.
- Uydu görüntüsü katmanı: Cesium ion'un varsayılan Bing/Sentinel katmanı ya
  da `Cesium.IonImageryProvider` — hangisi ion free tier'da sorunsuz
  çalışıyorsa onu kullan.
- 3D bina verisi (OSM Buildings) EKLEME — performans için gereksiz ağırlık,
  sadece arazi + görüntü yeterli.
- Gökyüzü/atmosfer: Cesium'un varsayılan atmosfer/gökyüzü kutusu açık kalsın
  (gerçekçilik için), ama gölge/gece ışıkları gibi ağır efektleri KAPAT.

## PERFORMANS KISITI (ÖNEMLİ)
Bu bileşen 4GB VRAM'li bir laptop GPU'da (RTX 3050 Laptop) çalışacak. Bu
yüzden:
- `maximumScreenSpaceError` değerini yüksek tut (düşük detay = daha az yük,
  örn. 8-16 arası, varsayılan 2'den çok daha hafif).
- Anti-aliasing/post-processing efektlerini kapat (`scene.postProcessStages`
  vb. varsayılan/kapalı bırak).
- Gölgeleri kapat (`viewer.shadows = false`).
- Hedef: orta seviye bir laptop GPU'da akıcı (en az ~20-30 fps) çalışmalı,
  fotogerçekçi ama hafif bir denge kur.

## PROP ŞEMASI (öneri, gerekirse uyarlayabilirsin)

```json
{
  "cesium_ion_token": "",
  "lat": 39.7767,
  "lon": 30.5206,
  "alt_m": 800,
  "roll_deg": 0,
  "pitch_deg": 0,
  "yaw_deg": 0,
  "camera_mode": "pov",
  "link_ok": true,
  "terrain_ready": null
}
```

- `alt_m`: metre, WGS84 elipsoid ya da arazi üstü — ikisini de destekleyebilirsen "arazi üstü göreli irtifa" tercih sebebi.
- `terrain_ready`: bileşenin kendi iç state'i, dışarıdan zorunlu değil.

## DAVRANIŞ / DÜRÜSTLÜK KURALLARI
- Cesium ion token boşsa VEYA terrain/imagery yüklenemezse (ağ hatası vb.),
  koyu arka plan üzerinde SADE bir durum mesajı göster
  ("CESIUM ION TOKEN GEREKLİ" ya da "ARAZİ VERİSİ YÜKLENEMEDİ") — asla boş
  siyah ekran ya da sahte/placeholder bir arazi GÖSTERME, bu proje gerçek
  veri ile sentetik veriyi asla karıştırmıyor.
- `link_ok=false` olduğunda görünümün üstüne (OSD zaten kendi LINK LOST
  uyarısını basacak, sen ekstra bir şey yapma) — sadece kamera pozisyonunu
  son bilinen konumda DONDUR, rastgele/varsayılan bir konuma SIÇRAMA.
- Yükleme sırasında (terrain/imagery ilk kez çekilirken) kısa bir "ARAZİ
  YÜKLENİYOR..." göstergesi olsun.

## GÖRSEL STİL
Cesium'un kendi varsayılan render'ı (gerçekçi uydu+arazi) olduğu gibi kalsın
— üstüne dashboard'un koyu/yeşil (#0a0c0f / #00e676) temasından bir şey
BOYAMA, gerçekçiliği bozar. Sadece yükleniyor/hata durumu mesajlarında bu
palet kullanılabilir (monospace font, koyu zemin, yeşil/kırmızı vurgu).

## ÇIKTI FORMATI
Önceki GCS panel ve OSD overlay export'larıyla aynı (`.dc.html`, kendi
kendine yeten dc-runtime + `support.js`), aynı klasör yapısıyla teslim et.

## NOT
Cesium ion'un ücretsiz bir hesap + API token'ı gerekiyor — bunu Yusuf kendisi
`cesium.com/ion` üzerinden alacak (Claude Design oluşturamaz), export'u
entegre ederken token eklenecek.

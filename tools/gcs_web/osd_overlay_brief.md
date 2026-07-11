# UAV-GCS OSD (On-Screen Display) Overlay — Bileşen Talebi

## BAĞLAM
Savunma sanayi tarzı bir Ground Control Station dashboard'unun parçası. Bir
gimbal kameradan gelen canlı video akışının ÜZERİNE bindirilecek, askeri/
taktik HUD tarzında saydam bir telemetri katmanı istiyorum (QGroundControl,
Mission Planner, ya da F-16 HUD görünümü referans alınabilir).

## TEKNİK ÇERÇEVE
- Bileşen, bir `<video>` veya `<img>` elemanının (canlı video kaynağı, prop
  olarak verilecek) TAM ÜSTÜNE mutlak konumlanan saydam bir SVG/canvas
  katmanı olmalı. Video kaynağının kendisini bu bileşen üretmeyecek —
  sadece üstüne bindirilecek overlay'i üretecek. Test için renkli bir
  placeholder `<div>` video yerine geçebilir.
- Tüm veriler data-props/computed props üzerinden parametrik olmalı
  (önceki panelde kullandığımız `{{ }}` mustache binding + `sc-if`/`sc-for`
  deseniyle aynı yapı).
- Responsive: video elemanı hangi boyutta olursa olsun overlay tam
  oturmalı (absolute positioning + %/vw-vh birimleri, sabit px değil).

## RENK/STİL
- Ana çizgi rengi: parlak yeşil `#00e676` (mevcut dashboard paletiyle
  tutarlı), ince çizgiler, hafif glow/text-shadow (gerçek HUD fosfor
  görünümü).
- LINK LOST durumunda tüm overlay kırmızıya (`#ff3b3b`) dönmeli ve
  kenarlarda yavaş pulse/blink efekti olmalı.
- Yazı tipi: monospace, teknik/askeri görünüm (mevcut panelde kullanılan
  fontla aynı aile).
- Arka plan tamamen saydam — video her zaman görünür kalmalı, overlay
  sadece çizgi/yazı.

## SEMBOLOJI VE YERLEŞİM

1. **Merkez**: sabit crosshair/reticle (basit + işareti, ortada, video ile
   birlikte hareket ETMEZ — sabit kalır çünkü kamera gimbal ile stabilize).

2. **Roll-compensated pitch ladder** (ortada, ufuk çizgisi + üstünde/altında
   10°'lik açı çizgileri) — roll prop'una göre TÜM ladder döner, pitch
   prop'una göre yukarı/aşağı kayar. (Gerçek attitude verisiyle beslenecek.)

3. **Üst-orta**: pusula/heading tape — yatay kayan şerit, mevcut heading'i
   ortada gösterir (N/E/S/W + derece işaretleri).

4. **Sol-üst köşe**: konum bilgisi — MGRS grid referansı + enlem/boylam,
   irtifa (metre).

5. **Sağ-üst köşe**: sistem saati (HH:MM:SS, canlı), "REC ●" kayıt
   göstergesi (kırmızı nokta + yanıp sönme), uçuş modu etiketi
   (örn. "HITL / MANUAL").

6. **Sol-alt köşe**: LINK durumu (OK yeşil / LOST kırmızı yanıp sönen),
   sinyal kalite çubukları (1-5 bar tarzı basit gösterge).

7. **Sağ-alt köşe**: gimbal açısı (pitch/yaw derece), zoom seviyesi
   (statik "1.0x" olabilir, gerçek zoom yoksa gösterme).

8. **Kenarlar**: köşe parantezleri (viewfinder/kamera çerçevesi hissi,
   dört köşede L-şeklinde ince çizgiler, tam kare çerçeve değil).

9. **Alt-orta**: küçük bir uyarı/durum satırı — "SIM" veya "GERÇEK" rozeti
   destekleyecek şekilde her bir veri grubu için opsiyonel küçük etiket
   alanı (örn. hız verisi yoksa "SPD: N/A" gösterilebilmeli, veri yokken
   sahte sayı üretilmemeli — tamamen boş/N/A durumu net görünmeli).

## PROP ŞEMASI (öneri, gerekirse uyarlayabilirsin)

```json
{
  "roll_deg": 0,
  "pitch_deg": 0,
  "yaw_deg": 0,
  "altitude_m": null,
  "lat": null,
  "lon": null,
  "mgrs": null,
  "link_ok": true,
  "flight_mode": "HITL / MANUAL",
  "gimbal_pitch_deg": null,
  "gimbal_yaw_deg": null,
  "recording": false,
  "timestamp": ""
}
```

## DAVRANIŞ
- Herhangi bir alan null/undefined gelirse o alan "N/A" veya "—" göstersin,
  ASLA rastgele/sahte bir sayı üretmesin (bu proje gerçek sensör verisiyle
  sentetik veriyi net ayırıyor, overlay bu prensibi bozmamalı).
- `link_ok=false` olduğunda TÜM overlay kırmızıya döner ve ortada büyük
  "LINK LOST" yazısı yanıp söner (video hâlâ görünür kalır, sadece overlay
  rengi değişir).

## ÇIKTI FORMATI
Önceki GCS panelinde kullandığımız export formatıyla aynı (`.dc.html`,
kendi kendine yeten dc-runtime + `support.js`).

# Fork kullanımı: gönderme, çekme, esas depodan güncelleme

Bu depo [alitalhq/ika-ros](https://github.com/alitalhq/ika-ros)'un fork'udur:
[MustafaBayrak1/ika-ros](https://github.com/MustafaBayrak1/ika-ros).
Kendi değişikliklerimiz fork'a gider, esas depodaki güncellemeler oradan çekilir.

| Remote | Adres | Ne için |
|---|---|---|
| `origin` | `MustafaBayrak1/ika-ros` | `git push` / `git pull` buraya |
| `upstream` | `alitalhq/ika-ros` | Yalnızca güncelleme almak için |

Kontrol:

```bash
git remote -v
```

## 1. İlk kurulum

**Yeni bir bilgisayarda (SSH anahtarı GitHub'a ekliyse):**

```bash
git clone git@github.com:MustafaBayrak1/ika-ros.git ~/ika-ros
cd ~/ika-ros
git remote add upstream https://github.com/alitalhq/ika-ros.git
```

**Robotta (Gemstone'da GitHub anahtarı yok, HTTPS kullanılır):**

```bash
git clone https://github.com/MustafaBayrak1/ika-ros.git ~/ika-ros
cd ~/ika-ros
git remote add upstream https://github.com/alitalhq/ika-ros.git
```

**Esas depodan klonlanmış mevcut bir kopyayı fork'a çevirmek:**

```bash
cd ~/ika-ros
git remote rename origin upstream
git remote add origin git@github.com:MustafaBayrak1/ika-ros.git   # robotta: https://github.com/MustafaBayrak1/ika-ros.git
git fetch origin
git branch -u origin/main main
```

## 2. Kendi değişikliklerimizi fork'a göndermek

```bash
git status                      # neler değişti
git add docs/BAYRAK/notlar.md   # ya da: git add -A
git commit -m "docs: kısa açıklama"
git push
```

Büyük ya da deneme niteliğindeki işler için ayrı dal açın:

```bash
git switch -c yeni-ozellik
# ... değişiklik, commit ...
git push -u origin yeni-ozellik
```

İş bitince GitHub'da fork sayfasından `main`'e Pull Request açıp birleştirin,
sonra yerelde:

```bash
git switch main
git pull
```

## 3. Fork'tan çekmek (ör. laptopta yapılanı robota almak)

```bash
cd ~/ika-ros
git pull
```

Robotta kod değiştiyse derleyip servisi yeniden başlatın
([native-kurulum.md](../native-kurulum.md#güncelleme)):

```bash
source /opt/ros/jazzy/setup.bash
MAKEFLAGS=-j2 colcon build --symlink-install --executor sequential --cmake-args -DBUILD_TESTING=OFF
sudo systemctl restart ika
```

Yalnızca belge (`docs/`) değiştiyse derleme gerekmez.

## 4. Esas depodan (upstream) güncelleme almak

Önce neler geldiğine bakın:

```bash
git fetch upstream
git log --oneline main..upstream/main     # bizde olmayan yeni commit'ler
```

Birleştirip fork'a gönderin:

```bash
git switch main
git merge upstream/main
git push origin main
```

**Kısa yol (web):** fork sayfasında **Sync fork → Update branch**, sonra yerelde
`git pull`.

Güncellemeden sonra robotta [3. bölümdeki](#3-forktan-çekmek-ör-laptopta-yapılanı-robota-almak)
`git pull` ve derleme adımlarını uygulayın.

### Çakışma çıkarsa

`git merge` "CONFLICT" derse:

```bash
git status                      # çakışan dosyalar "both modified"
# dosyayı açın, <<<<<<< ======= >>>>>>> arasını düzeltip işaretleri silin
git add <dosya>
git commit                      # birleştirmeyi tamamlar
git push origin main
```

Vazgeçmek için: `git merge --abort`.

Çakışmayı önlemenin en kolay yolu: kendi notlarımızı esas depodaki dosyalara
değil, **`docs/BAYRAK/`** altındaki yeni dosyalara yazmak.

## 5. Esas depoya katkı göndermek (isteğe bağlı)

Bir düzeltme esas depoya da gitsin istenirse, değişikliği güncel `upstream/main`
üzerinden açılan ayrı bir dalda yapın:

```bash
git fetch upstream
git switch -c duzeltme upstream/main
# ... değişiklik, commit ...
git push -u origin duzeltme
```

Sonra GitHub'da bu daldan **alitalhq/ika-ros**'a Pull Request açın.
`docs/BAYRAK/` gibi bize özel dosyaları bu dallara koymayın.

## Notlar

- Robotun çizgi sensörü kalibrasyonu, lidar seçimi ve kayıtlı haritalar depoda
  değil, robotta (`/root/.ika`, `/root/ika_maps`) durur; `git pull` bunlara
  dokunmaz.
- `main` dalına `git push --force` yapmayın.
- Fork'ta GitHub Actions (otomatik testler) varsayılan olarak kapalıdır; fork
  sayfasındaki **Actions** sekmesinden bir kez açılabilir.

# Mô phỏng biển thoát hiểm thông minh: logic đóng/mở cửa khi cháy

Tài liệu này dành cho trưởng nhóm: mô tả hệ thống mô phỏng làm gì, logic cửa hoạt động ra sao, đã kiểm chứng đến đâu, và còn những điểm nào cần quyết định.

## 1. Mục đích

Mô phỏng sơ tán đám đông khỏi tòa nhà khi có cháy bằng mô hình Social Force. Mỗi cửa thoát hiểm có một **biển thông minh** gắn camera và bộ đếm người bằng AI:

- **Đông người trước cửa:** biển chuyển **đỏ**, cửa **đóng**, người phải đi cửa khác để tránh chen ép.
- **Ít người:** biển **xanh**, cửa **mở**.

Mục tiêu của mô phỏng là kiểm tra logic này có giảm nguy cơ chen ép mà không làm thoát chậm đi quá nhiều hay không, trước khi triển khai thiết bị thật.

## 2. Camera mô phỏng (OmniVision OV5648)

| Thông số | Giá trị | Ghi chú |
|---|---|---|
| Góc nhìn ngang | 58,6° | Nón hướng từ cửa ra hành lang, không nhìn 360° như bản gốc |
| Tầm đếm | 8 m | Có thể chỉnh 8 đến 10 m; trên 10 m chưa nên tin độ chính xác |
| Vùng mù dưới camera | tính từ chiều cao lắp, góc chúc, góc nhìn dọc 45,3° | **Chiều cao 2,5 m và góc chúc 30° là giá trị giả định, cần thay bằng số đo thật** |
| Chu kỳ đọc | 1 giây | Bộ đếm AI trên thiết bị nhúng không đọc 10 lần mỗi giây |
| Sai số đếm | ±10% | Giả định, không còn "đếm hoàn hảo" |
| Làm mượt | trung bình 3 giây | Chống một khung hình nhiễu làm đóng cửa nhầm |

Giá trị đưa cho cửa là **mật độ (người/m²)** trên diện tích vùng nhìn, không phải số người, vì vùng nhìn mỗi cửa có diện tích khác nhau (khoảng 23 đến 30 m²).

## 3. Logic cửa

Mỗi cửa là một biển **độc lập**, có 3 trạng thái:

| Màu | Trạng thái | Cửa |
|---|---|---|
| Xanh | OPEN | Mở, đi qua được |
| Cam | CLOSING | Cảnh báo 3 giây, **vẫn đi qua được** |
| Đỏ | CLOSED | **Đóng hẳn**, không ai đi qua được |

```
OPEN --(mật độ > 0,7 và đã mở >= 5 s)--> CLOSING --(hết 3 s, vẫn đông)--> CLOSED
                                            |                                 |
                                            +--(đám đông tan)--> OPEN <-------+
                                  CLOSED --(đã đóng >= 10 s và mật độ <= 0,4)--> OPEN
                                  CLOSED --(đã đóng >= 20 s, bất kể đông hay không)--> OPEN
```

**Các quy tắc đi kèm**

| Quy tắc | Giá trị | Lý do |
|---|---|---|
| Ngưỡng đóng | mật độ > 0,7 người/m² (khoảng 20 người trong vùng nhìn) | Ngưỡng mật độ, không phải số người |
| Ngưỡng mở lại | mật độ <= 0,4 người/m² | Thấp hơn ngưỡng đóng để tránh chớp tắt |
| Đóng tối thiểu | 10 giây | |
| **Mở cưỡng bức** | sau 20 giây đóng | Chống kẹt: cửa không bị đóng vô hạn dù đám đông không tan |
| Mở tối thiểu | 5 giây sau khi mở | Chống đóng/mở liên tục |
| Cảnh báo trước khi đóng | 3 giây (cam) | Người sát cửa kịp đi qua |
| **Không đóng cửa cuối cùng** | luôn còn ít nhất 1 cửa khác đi được | Tránh cả 4 cửa cùng đóng; cần một tin nhắn giữa các biển qua hub |
| Người trong khe cửa | được cho qua khi cửa đóng | Không để người bị kẹp |

**Hành vi của người khi cửa đóng**

- Người **không được báo** ngay. Họ chỉ phát hiện cửa đóng khi đến gần trong 15 m (tính theo đường đi).
- Sau đó họ mất 1 đến 3 giây phản ứng, rồi đổi sang cửa gần nhất còn mở.
- Người ở xa vẫn đi tiếp về phía cửa đỏ cho đến khi lại gần.

**Chế độ thử nghiệm:** `--mode divert`: biển đỏ nhưng cửa vẫn mở, chỉ chuyển hướng người chưa vào hàng. Không phải mô hình chính, chỉ để so sánh.

## 4. Kịch bản kiểm thử

Cửa vẽ trong bản đồ gốc rộng 5,6 m, rộng hơn nhiều so với cửa thoát hiểm thật (1,0 đến 2,4 m), nên ít khi quá tải. Vì vậy có thêm các kịch bản **quá tải**:

| Kịch bản | Mô tả |
|---|---|
| `uniform_1m2` / `uniform_1m0` | Cửa 1,2 m / 1,0 m, 300 người phân bố đều |
| `left_1m2` / `left_1m0` | Cửa 1,2 m / 1,0 m, 300 người dồn vào nửa trái tòa nhà (hai cửa bên phải vắng) |

Thước đo: thời gian thoát hết (T100), lực nén đỉnh, và **số người-giây bị ép trên 1000 N** (ngưỡng này là giả định).

## 5. Cách chạy

```
pip install -r requirements.txt
python -m unittest discover -s tests              # 23 test, kỳ vọng "OK"
python scenarios.py                                # so sánh tất cả kịch bản và cấu hình, ghi results/scenarios.csv
python run.py --agents 300 --door-width 1.2 --spawn-x-max 60 --save qua_tai.mp4   # một kịch bản có video
```

Kết quả mỗi lần chạy `run.py` được lưu vào `results/<ngày-giờ>/` gồm `summary.json`, `timeseries.csv`, `timeseries.png`. Xem thêm `python run.py --help`. Toàn bộ giá trị mặc định nằm trong `sfm/config.py`.

## 6. Kết quả đã chạy

300 người dồn nửa trái, 3 lần chạy ngẫu nhiên mỗi cấu hình. **Hai nhóm chạy ở hai đợt khác nhau, nên mỗi cấu hình chỉ so với "không cổng" cùng đợt.**

| Đợt | Cửa | Cấu hình | Số lần thoát hết | T100 (giây) | Người-giây bị ép nguy hiểm |
|---|---|---|---|---|---|
| A | 1,2 m | Không cổng | 3/3 | 82 | 41 |
| A | 1,2 m | **Chặn cửa** | 2/3 | 136 | 21 |
| A | 1,0 m | Không cổng | 3/3 | 110 | 72 |
| A | 1,0 m | **Chặn cửa** | 0/3 (trong 200 s) | không xong | 30 |
| B | 1,2 m | Không cổng | 3/3 | 83 | 49 |
| B | 1,2 m | Chuyển hướng (thử nghiệm) | 3/3 | 112 | 23 |
| B | 1,0 m | Không cổng | 3/3 | 113 | 81 |
| B | 1,0 m | Chuyển hướng (thử nghiệm) | 3/3 | 134 | 22 |

Các kết luận hiện có:

1. **Đóng cửa giảm khoảng một nửa lượng người bị ép**, nhưng **thoát chậm hơn nhiều**, thậm chí có lần không ra hết trong 200 giây. Người xếp hàng trước cửa đóng rồi đổ dồn vào khi cửa mở lại.
2. Chuyển hướng (cửa đỏ nhưng vẫn mở) giảm nguy hiểm tương tự, thoát chậm hơn khoảng 30%, nhưng không đúng mô hình "đỏ là đóng".
3. Với cửa vẽ gốc 5,6 m, 200 đến 500 người phân bố đều: logic mới không bao giờ đóng cửa, vì cửa rộng nên không quá tải. Logic cũ (đếm 25/15 trên vòng tròn 10 m, đóng tức thì) thì có lúc đóng đến 26 lần và 500 người không ra hết.
4. Chọn cửa theo chi phí (đi bộ cộng thời gian chờ) cho kết quả y hệt "không cổng", vì cửa bên phải cách hơn 100 m, nên không ai đổi cửa. Tính năng này mặc định tắt.

## 7. Hạn chế và giả định

- Mô hình đếm của AI được giả lập bằng sai số ±10%, **chưa dùng bộ đếm thật** (CSRNet hoặc DM-Count).
- Chiều cao lắp và góc chúc camera chưa có số đo thật.
- Hệ số lưu lượng cửa (1,5 người/giây/mét) và ngưỡng nguy hiểm (1000 N) là giả định.
- **Camera cửa E2 và E4 gần như không thấy ai**, vì người đi sát tường rồi rẽ vào cửa từ bên cạnh, nằm ngoài nón nhìn. Hai cửa này hầu như không bao giờ đóng dù có chen chúc.
- Mỗi cấu hình mới chạy 3 lần ngẫu nhiên, đủ để thấy xu hướng nhưng chưa đủ để kết luận về mức chênh lệch nhỏ.
- Mô phỏng chưa mô hình hóa khói, lửa, hay lỗi cảm biến và mất điện.


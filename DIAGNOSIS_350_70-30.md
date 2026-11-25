# Chẩn đoán 117 người trong lượt 650 người, 70/30

Phạm vi: bản PNG 1382×729 trước khi thay bằng 1.jpg. Chỉ dùng các số liệu dưới đây cho bản đồ PNG cũ.

## Nguồn đối chiếu

- MP4 gốc: `D:/bach/KLTN_new_data/social-force-smart-exit-sign/results/stair_study/350_70-30/seed0/OFF_vs_ON.mp4`. SHA-256 trước khi thay: `45b075f986f33238a5e945843798b3debb4edf3eb41ca63a31c4e0ebe23f9f60`.
- Git HEAD: `6cb4552633abdcdd769717392ffbcdbc7cb221c7` với mã nghiên cứu đã sửa trong worktree. Mã lực `sfm/crowd.py` trước sửa có SHA-256 `5db2e48288f8af8901caf1aa279ae70a18983132c14e10c5ea841954420cc59b`. Trường điều hướng dùng cache v4, SHA-256 mã điều hướng `b88431c5e0949182e07a5583e6276c3c2ea55f5b8d13139137a80bf8f4b4e87e`.
- Seed 0, quota 455/195. SHA-256 trạng thái ban đầu `f6defeef782fb1185b8034cb7a491aabb53ace3e96bea231a17c91f627e2e0a1`. MP4 gốc chứa hình ảnh, nên ID được khôi phục bằng replay tất định. So khung OFF tại t=150, 575 và 600 s cho sai khác trung bình 1,49/255 mỗi kênh sau giải mã. Replay khớp 117 người còn lại, trung bình giới hạn 147,61 s và 0 lượt đổi cửa.
- `1.png`, 0,05 m/px, lưới điều hướng 0,1 m, Euler bán ẩn với dt 0,01 s, thời hạn 600 s. Người 80 kg, đường kính 1 m, tốc độ mong muốn 1,5–2,0 m/s, τ 0,5 s và hệ số giới hạn tốc độ 1,3. SFM dùng A và A tường 2.000 N, B và B tường 0,08 m, k 120.000 kg/s², κ 240.000 kg/(m·s), cutoff cấu hình 2 m, skin 0,5 m và noise 0. Mốc đến là 1 m sâu trong vùng đích cùng tầng. Python 3.10.0, NumPy 2.2.6, OpenCV 5.1.0.

## Từng cụm ở t=150 s

| Nhãn phòng | Số người | Tâm cửa (x, y), m | Khe mở nhỏ nhất | Đến đích khi chạy riêng từng người | Qua cửa phòng khi giữ cả cụm, 35 s |
|---|---:|---|---:|---:|---:|
| 3 | 21 | (23,250; 5,000) | 1,20 m | 21 | 0 |
| 4 | 12 | (28,325; 5,000) | 1,15 m | 12 | 0 |
| 20 | 12 | (9,950; 23,650) | 1,20 m | 12 | 0 |
| 26 | 19 | (9,850; 31,450) | 1,20 m | 19 | 0 |
| 27 | 23 | (23,300; 31,450) | 1,20 m | 23 | 0 |
| 28 | 30 | (28,475; 31,450) | 1,15 m | 30 | 0 |

Khe mở đo trên raster qua toàn chiều dày vách. Khoảng cách từ tâm khe tới tường tại sáu cửa xấp xỉ 0,575 m, đủ cho bán kính 0,50 m. Tất cả 117 điểm có đường hữu hạn trong đồ thị xét bề rộng. Replay giữ vị trí, vận tốc, cửa mục tiêu và tốc độ mong muốn ở snapshot. Lượt riêng chỉ loại tương tác với các người khác. Lượt cả cụm giữ tương tác nội cụm.

| Phòng | ID |
|---|---|
| 3 | 34, 55, 58, 106, 140, 182, 184, 199, 216, 226, 241, 333, 423, 460, 474, 506, 522, 523, 526, 597, 620 |
| 4 | 17, 87, 136, 166, 186, 217, 302, 330, 365, 426, 477, 628 |
| 20 | 16, 88, 163, 204, 231, 237, 280, 301, 369, 422, 473, 636 |
| 26 | 13, 32, 48, 51, 56, 66, 70, 100, 109, 183, 215, 245, 325, 326, 445, 467, 529, 558, 566 |
| 27 | 6, 61, 75, 104, 120, 132, 153, 157, 178, 222, 260, 275, 314, 331, 346, 349, 363, 378, 465, 513, 515, 548, 555 |
| 28 | 37, 52, 64, 99, 102, 108, 111, 137, 138, 148, 150, 162, 187, 233, 240, 248, 282, 309, 317, 344, 370, 395, 403, 404, 527, 565, 580, 612, 622, 625 |

## Đo lực và sửa hành vi

Đo 42.500 bước từ t=150 đến 575 s. Độ dịch chuyển cực đại là 7,11×10⁻¹⁵ m. Trace chi tiết t=575–576 s lưu đủ 100 bước dt cho từng người. Vận tốc cực đại là 3,71×10⁻¹³ m/s và hợp lực cực đại là 1,74×10⁻¹⁰ N. Ví dụ ID 150 ở phòng 28 có lực điều hướng (234,7; 54,0) N, lực người–người (−817,4; −1.406,3) N và lực tường (582,7; 1.352,4) N. Hợp lực triệt tiêu tại sai số làm tròn. Hệ giữ nguyên các ID và vị trí đến sai số dấu phẩy động từ các snapshot 150, 575 và 600 s.

Nhiều người cùng tiến vào một khe chỉ đủ một thân người tạo cân bằng lực tập thể. Thử cho người phía sau đứng đợi hoặc lùi thẳng vẫn giữ được thế kẹt ở phòng 28. Bản sửa cho một người dẫn tiến vào cửa, nhóm phía sau đứng đợi và người cạnh lối dạt sang bên jamb khi đủ khoảng trống thân người. Chỉ kích hoạt ở phòng có một cửa ra kiểu một hàng khi cả nhóm dưới 0,05 m/s trong 2 s. Nếu lại đứng yên, ưu tiên một người khác gần đầu hàng. Quyết định cập nhật mỗi 0,1 s. Ở lần chạy trung gian, ID 565 bắt đầu rẽ khi còn trong khe phòng 28. Đo một mét tới trước cho khe hở 0,456 m theo hướng rẽ và 0,575 m theo hướng thẳng, so với bán kính 0,50 m. Người dẫn giữ hướng vuông góc với cửa nếu cả đoạn một thân người phía trước đủ thoáng. Các lực, kích thước thân người, bước tích phân, bản đồ và chính sách tín hiệu giữ cùng tham số ở cả OFF và ON.

Replay toàn bộ snapshot 117 người từ t=150 s qua bản sửa. Sau 30 s đã có người qua ở cả sáu phòng, theo thứ tự bảng là **11, 10, 10, 11, 11, 14**. Người cuối tới vùng đích tại t=241,59 s. Kiểm thử hồi quy lưu cụm 30 người phòng 28 tại `tests/fixtures/room28_t150.json`.

Dữ liệu cục bộ gồm `cache/jam_350_70-30/metadata.json`, các snapshot `t150.npz`, `t575.npz`, `t600.npz` `long_trace_summary.json` và `trace.npy`. Trace có trục (bước thời gian, trường, ID, tọa độ) với các trường theo thứ tự vị trí, vận tốc, hợp lực và hướng mong muốn. Có thể kiểm thử bằng `python -m unittest discover -s tests -v`.

## Kiểm tra bản sửa từ trạng thái ban đầu

Chạy lại seed 0 trước khi xuất video. Cả 650 người tới vùng đích. Thời điểm đến cuối của từng nhóm ID gốc theo thứ tự bảng là **138,72 / 94,96 / 140,11 / 175,15 / 181,18 / 134,78 s**. T90 là 121,44 s và T100 là 181,18 s. Đỉnh lực tiếp xúc tính theo bộ ghi của mô hình tăng từ 4,95 lên 5,87 kN, và phơi nhiễm trên ngưỡng 1.000 N tăng từ 85,7 lên 151,8 người·giây. Các tham số lực giữ nguyên; việc hết kẹt cần được đọc cùng chỉ số tiếp xúc này.

## Camera sau khi xử lý kẹt

Tại t=575 s của lượt gốc cả bốn camera đếm 0 người, dù còn 117 người trong phòng. Mặt bằng và tường giới hạn vùng quạt về hành lang tới 2,425 m² cho E1/E2 và 2,560 m² cho E3/E4. Cấu hình 11,2 m có tầm hiệu dụng 6,081 m. Mỗi camera quan sát nhiều nhất 2 người khi chạy lại từ đầu. Đỉnh sau làm mượt theo E1–E4 là **0,825 / 0,825 / 0,781 / 0,781 người/m²**. Với ngưỡng đỏ trên 3,0, số người trung bình trong vùng nhìn phải vượt lần lượt 7,275 và 7,680. Vì thế không có tín hiệu đỏ để kích hoạt quyết định chuyển. Kiểm tra hình học trên lưới 0,1 m cũng ghi nhận 0 điểm mẫu đọc đồng thời hai biển cửa trong 16 m.

Chỉ xuất lại lượt này bằng `python scenarios.py --only 350_70-30`.

MP4: 414 frames, 10 FPS, 1536 x 560, SHA-256 `8531478564ea186c4718d837d30c355a9a19bfe3c4960f3546d34a8dc0a32fa3`, mtime `2026-11-19T16:29:02+07:00`. OFF and ON: COMPLETED, T90 121.44 s, T100 181.18 s.

## Đối chiếu ngưỡng và tỷ lệ camera

Commit `6cb4552` thay ảnh 1479×615 bằng ảnh 1382×729 và đổi tỷ lệ từ 0,1 thành 0,05 m/px. Ảnh hiện tại quy đổi thành 69,10×36,45 m ở tỷ lệ đang dùng, hoặc 138,20×72,90 m nếu áp tỷ lệ cũ. Khe phòng 23–24 px tương ứng 1,15–1,20 m và khe thoát 47–48 px tương ứng 2,35–2,40 m theo tỷ lệ đang dùng. Repository chưa có bản đo thực địa để xác nhận kích thước tuyệt đối.

Với HFOV 58,6°, VFOV 45,3°, cao 2,5 m và nghiêng 45°, giới hạn gần/xa là 1,028/6,081 m. Sau khi áp bán kính 11,2 m, hình quạt vành khăn trước khi xét tường là 18,367 m². Phần pixel sàn trong hình quạt theo E1–E4 là 2,425 / 3,735 / 2,560 / 3,805 m². Sau raycast loại điểm bị che, diện tích đếm còn 2,425 / 2,425 / 2,560 / 2,560 m². Pixel xanh lam trong thư mục `results/stair_study/350_70-30/seed0/camera_masks` đúng bằng mặt nạ dùng để đếm.

Cả hai profile giữ SHA-256 khởi tạo ở trên và cho OFF/ON cùng T90 121,44 s, T100 181,18 s, trung bình 64,11 s và 0 người còn lại. Profile tham chiếu 3,0/1,0 không có lần RED, lượt đọc RED, lượt đủ điều kiện hay lần chuyển nào. Với profile thử nghiệm 0,7/0,4:

| Camera | Số lần RED | Tổng giây RED | ID thấy đỏ | Đủ điều kiện | Chuyển |
|---|---:|---:|---:|---:|---:|
| E1 | 1 | 90 | 174 | 0 | 0 |
| E2 | 1 | 10 | 25 | 0 | 0 |
| E3 | 4 | 77 | 202 | 0 | 0 |
| E4 | 1 | 10 | 38 | 0 | 0 |

Đếm người xem là số ID duy nhất trên các kỳ camera 1 s. Cả 439 người được ghi đều đang nhắm chính cửa đỏ mình nhìn thấy. Số đủ điều kiện lấy tại lượt quyết định thật. Lưới đường nhìn 0,1 m ghi nhận không có điểm nhìn đồng thời hai biển trong tầm đọc 16 m. Bốn đỉnh mật độ làm mượt vẫn lần lượt là 0,8247 / 0,8247 / 0,7812 / 0,7812 người/m². Bộ ngưỡng này chỉ dùng để thử phản ứng mô phỏng.

Experimental MP4: 414 frames, 10 FPS, mtime `2026-11-19T16:56:37+07:00`, SHA-256 `fc361374bab55ae3ab3b86237605c96addcf6490da125c66eae79fff3850f987`.

## Thử nghiệm biển tại cửa nhận trạng thái qua mạng

Với cùng SHA-256 khởi tạo, profile tham chiếu 3,0/1,0 vẫn có 0 lần RED và 0 lần chuyển. Profile thử nghiệm 0,7/0,4 ghi nhận 436 ID đọc biển đỏ tại kỳ camera, 159 ID đủ điều kiện tại lần xét thật và 100 ID chuyển tổng cộng 101 lần. OFF hoàn tất với T90/T100 121,44/181,18 s, trung bình 64,11 s. ON hoàn tất với T90/T100 137,54/576,93 s, trung bình 71,79 s. MP4 1.205 khung thay tại đường dẫn cũ lúc 17:07:35 ngày 19/11/2026, UTC+7. Log theo ID và lý do nằm cạnh video trong `guidance_audit.json`.

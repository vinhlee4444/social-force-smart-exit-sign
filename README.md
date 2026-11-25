# So sánh SFM OFF/ON

Chạy từ thư mục gốc. Trên Windows có thể dùng `venv\Scripts\python.exe` thay cho `python`.

```powershell
python -m pip install -r requirements.txt
python -m unittest discover -s tests -v
python scenarios.py --preflight
python scenarios.py --smoke
python scenarios.py --only 100_20-80
python scenarios.py
```

Runner kiểm tra cả sáu cấu hình và xuất mỗi cặp khả thi vào `results/stair_study/<cau_hinh>/seed0/OFF_vs_ON.mp4`. Video lấy mẫu mỗi 0,5 s mô phỏng, phát 10 FPS (5×), giữ khung cuối của bên hoàn tất trước và hiển thị bảng kết quả trong 5 giây phát. Runner kiểm tra số khung, FPS và bảng kết quả cuối trên MP4 tạm trước khi thay tệp đích.

- Dùng `1.jpg` (1188×730 pixel), 0,05 m/px, bước vật lý 0,01 s, thời hạn 600 s. Hoàn tất khi tâm người vào vùng đích cùng tầng, ít nhất 1 m sau mép trong cửa. Mô hình chưa có cầu thang, vùng đệm hoặc hành trình ra khỏi toàn bộ tòa nhà.
- Chia trái/phải tại **x = 29,50 m** theo trung điểm bề rộng sàn dùng được trên JPEG. Tìm được 404/365 chỗ theo thứ tự trái/phải với khoảng cách tâm ≥ 1,05 m và khe hở thân tới tường ≥ 0,10 m lúc sinh. Nhãn `350_*` dùng 350 người với quota 70/280 và 245/105. Nhãn `600_*` giữ tổng quota 1.000 người. Cận trên trái/phải là 547/503. Hai lượt 1.000 người vượt cận trên ở phía cần đặt người.
- Hai bên chọn cửa gần nhất ban đầu theo chiều dài đường đi trên lưới 0,1 m có xét bán kính cơ thể 0,5 m. Runner so SHA-256 của trạng thái ban đầu. OFF giữ cửa mục tiêu và đi theo trường hướng SFM có trọng số tránh tường.
- Camera/bộ đếm giả lập lấy mẫu mỗi 1 s, sai số giả lập 0%, lấy trung bình tối đa 3 mẫu gần nhất, đơn vị người/m². HFOV 58,6°, VFOV 45,3°, cao 2,5 m và nghiêng 45° là tham số minh họa. Bán kính 11,2 m bị hình học giới hạn còn 6,081 m, vùng mù 1,028 m. Tường tiếp tục cắt vùng quạt, còn 2,425 m² mỗi camera E1/E2 và 2,555 m² ở E3 và 2,560 m² ở E4. OFF hiển thị mật độ để đối chiếu và tắt tín hiệu.
- ON khởi tạo đỏ nếu mẫu đầu > 3,0, xanh nếu ≤ 3,0. Sau đó đổi sang đỏ ở > 3,0 sau ít nhất 5 s xanh và đổi sang xanh ở ≤ 1,0 sau ít nhất 10 s đỏ. Giá trị giữa hai ngưỡng giữ màu. Cửa vật lý luôn mở.
- Sinh viên ở cả hai chế độ biết vị trí và đường đi tới mọi cửa. Sau mỗi kỳ camera, mô hình mạng/hub chuyển đồng bộ trạng thái đỏ/xanh hợp lệ đến các bảng chỉ dẫn gắn ở cửa. Bảng đỏ hiển thị mã cửa xanh được khuyến nghị và mũi tên. ON chỉ đọc bảng của cửa mục tiêu trong 16 m khi có đường nhìn trực tiếp tới bảng đó. Sinh viên chọn trong các cửa được khuyến nghị theo đường hợp lệ ngắn nhất, chấp nhận đi thêm tối đa 30 m, xác suất 0,6 mỗi lượt xét, phản ứng 1–3 s có kiểm tra lại, xét lại sau 5 s và cooldown 10 s. Khi đã vào vùng cam kết 5 m, agent giữ cửa. Video khoanh người vừa chuyển và hiện ID cùng cửa cũ → cửa mới trong 3 giây mô phỏng. Phần cứng cần kết nối giữa các cửa và bảng có khả năng hiện mã/tên cửa thay thế cùng mũi tên. Một bóng đỏ/xanh đơn lẻ chỉ cung cấp trạng thái.
- T90/T100 dùng thời điểm đến vùng đích. Trung bình giới hạn dùng 600 s cho mỗi người chưa đến khi hết giờ. Chẩn đoán trong console đếm số agent nằm ngoài đồ thị đủ bề rộng và số agent tiến dưới 0,5 m trong 15 s, tách nhóm gần người khác (có thể đang chờ) và nhóm cô lập.

### Kiểm tra map hiện tại

`python map_check.py` lưu ảnh debug, các bề rộng cửa, quỹ đạo mẫu và kết quả kiểm tra từng phòng tại `results/map_check/<12-ký-tự-đầu-SHA-256>/`. Ảnh JPEG 1188×730 giữ bề rộng cửa theo pixel của ảnh PNG cũ nhưng cắt sảnh phía đông, kèm lệch raster nhỏ. Tỷ lệ 0,05 m/px giữ theo bề rộng cửa tham chiếu và chưa được khảo sát thực địa. Hai lối mở vào phần sảnh bị cắt và dải pixel ngoài vách đông được đóng biên trong mô hình. Bốn cửa tới vùng đích giữ vị trí theo các khe còn nhìn thấy.

Các test dùng snapshot cũ tải riêng `tests/fixtures/old_floor_6cb4552.png` với cấu hình cũ. Điều hướng trên JPEG dùng khóa cache xét SHA-256 ảnh, sàn đã xử lý, tỷ lệ, các vùng cửa/đích và cấu hình lưới. SDF, nhãn phòng, ROI và vị trí spawn được dựng lại khi chạy. Video mới ghi tên và SHA-256 map ở bảng cuối. Các đoạn báo cáo 650 người dưới đây thuộc PNG cũ.

#### Kiểm chứng trước khi thay MP4

Runner ghép là `scenarios.py`. Đường dẫn tương đối của `--map` tính từ thư mục repository. Có thể dùng `--expect-map-sha256` để khóa bytes của ảnh. Runner in đường dẫn tuyệt đối và hash ảnh thực đọc. Tùy chọn `--rebuild-nav` bỏ qua cache đọc và dựng lại route fields theo map. SDF, phòng, ROI và spawn được tạo trong phiên chạy.

```powershell
python scenarios.py --map 1.jpg --rebuild-nav --only 100_20-80 --threshold-profile experimental --preview
python scenarios.py --map 1.jpg --rebuild-nav --only 100_20-80 --threshold-profile experimental
```

`preview.png` dùng cùng renderer với video. MP4 được ghi vào tệp tạm cùng thư mục và kiểm tra giải mã đủ khung, khớp preview và bảng cuối trước khi thay `OFF_vs_ON.mp4`. `first_frame.png` là khung đầu giải mã từ chính dữ liệu MP4 đã công bố. Nếu chỉ định quota không khả thi, runner ghi đường dẫn bị bỏ qua và trả mã lỗi. `run.py` là lối chạy độc lập với runner ghép.

### Thử ngưỡng camera 350 người, 70/30 trên JPEG hiện tại

Hai profile dùng cùng seed 0 và mô hình vật lý. `reference` là mặc định (3,0/1,0 người/m²). `experimental` dùng 0,7/0,4 người/m² để kiểm tra phản ứng trong mô phỏng.

```powershell
python scenarios.py --only 350_70-30 --headless --camera-masks results/stair_study/350_70-30/seed0/camera_masks
python scenarios.py --only 350_70-30 --headless --threshold-profile experimental
python scenarios.py --only 350_70-30 --threshold-profile experimental
```

Pixel xanh lam trong mỗi PNG đúng bằng ROI dùng để đếm. Runner đếm ID duy nhất nhìn trực tiếp thấy RED tại kỳ lấy mẫu camera 1 s, tách nhóm đang nhắm chính cửa đó. Số đủ điều kiện lấy tại các lượt quyết định thật, trước khi bốc xác suất. Lệnh cuối kiểm tra MP4 tạm rồi thay video ghép ở đường dẫn gốc.

Runner xuất `guidance_audit.json` cạnh MP4, ghi ID, nguồn tại cửa, thời điểm bản tin camera, cửa đi và lý do quyết định.

### Ablation điều hướng và tránh đối đầu

Lượt 650 người, quota 455/195, seed 0 trước đây dùng PNG cũ. Để so ba nhánh headless trên JPEG hiện tại, chạy `python ablation.py --only 100_20-80` hoặc chọn một kịch bản 350 người. Cả ba dùng cùng mô hình tránh đối đầu và kiểm tra SHA-256 khởi tạo. Nhánh OFF tắt chỉ dẫn. Nhánh `congestion_guidance` dùng báo cáo GREEN để chọn cửa thay thế. Nhánh `covered_routes` yêu cầu thêm độ phủ camera cho 10 m cuối của tuyến tiếp cận. Hai nhánh ON dùng profile thử nghiệm 0,7/0,4. Số liệu từng ID và tổng hợp ghi trong `results/stair_study/<kịch_bản>/seed0/ablation`.

Khi hai người có hướng đối lập và sắp gặp nhau, quy tắc vận động đổi hướng mong muốn sang bên phải nếu đoạn dịch chuyển còn đủ bề rộng cơ thể. Vận tốc mong muốn và các hệ số lực giữ nguyên. Cửa hẹp dưới khả năng vượt nhau cần một chiến lược nhường khác.

`covered_routes` chỉ cung cấp khuyến nghị khi camera phủ các ô đi được trên đoạn tiếp cận cần quan sát. Hub chặn hướng ngược trên các đoạn chung của tuyến mà chính nó đã quảng bá. Hub giữ các hướng đã quảng bá đến hết lượt vì camera tại cửa chưa thể xác nhận tuyến đã rỗng. Quy tắc này không quan sát được dòng người tự chọn đường ngoài ROI. Nếu triển khai đo tải trên đoạn bị che, cần khảo sát camera trần hoặc cảm biến đếm hai chiều ở từng đoạn, gắn trạng thái độ phủ và thời điểm đo vào bản tin hub. Ablation này chỉ dùng bốn camera hiện có.

### Thử policy chuyển trước điểm phân nhánh

`python ablation.py --only 350_20-80 --new-policy forward_branch` ghép headless ba nhánh OFF, ON cũ và ON thử nghiệm. `forward_branch` chỉ xét cửa xanh nếu hai đường ngắn nhất trên lưới đủ rộng còn chung ít nhất 1 m trước khi tách trong 8 m phía trước, và nhánh mới không chĩa ngược lại. Đi thêm tối đa 50% phần đường gốc còn lại, đồng thời giữ trần 30 m. Điều kiện được kiểm tra lại khi hết thời gian phản ứng. Đây là bộ lọc thử nghiệm trên bản đồ tĩnh. Hai profile camera, P và mô hình vận động giữ nguyên.

```powershell
python ablation.py --only 350_70-30 --new-policy route_compatible
python scenarios.py --only 350_70-30 --threshold-profile experimental --guidance-policy route_compatible
```

`route_compatible` giữ điều kiện nhánh phía trước và giới hạn đi vòng của `forward_branch`. Sau điểm tách, hub so tuyến thay thế với tuyến gốc tương ứng và các tuyến đã khuyến nghị. Hai tuyến gần song song, ngược hướng, chiếu chồng nhau theo chiều hành lang, cách nhau tối đa 2 m và thấy nhau qua vùng sàn tạo xung đột. Agent xét nhánh từ vị trí và hướng của mình khi đọc được biển. Hub ghi các hướng đã phát đến hết lượt vì camera cửa chưa xác nhận được hành lang đã trống. Hub dùng báo cáo màu camera, tuyến tĩnh và lịch sử khuyến nghị. Trong triển khai, thiết bị phải hiển thị chỉ dẫn có tên hoặc mũi tên tại cửa và gắn chỉ dẫn với nhánh tiếp cận tương ứng.

Ablation ghi thời gian và chiều dài thực đi theo ID cùng bảng tổng hợp vào `results/stair_study/350_70-30/seed0/route_compatible_ablation`. Tùy chọn `--verify-current-video` kiểm tra MP4 và audit của `congestion_guidance` trước khi thay video. Fixture `counterflow_350_70-30_t64.json` lưu cụm hành lang tại giây 64. Test đối chiếu thành phần lực và dùng cùng mô hình cho dòng một chiều và hai chiều.

## Ghi chú lịch sử

Phần dưới ghi lại các giai đoạn cũ của repository. Dùng cấu hình và lệnh ở trên để tái lập phép so sánh hiện tại.

### 1. Cài đặt: 
requirements.txt, nhẹ thôi ae ạ

### 2. Giải thích

hiện đã vẽ xong bản đồ 2D (anh em xem file 1.PNG và chạy thử grid_wall.py - file này tự đổi ảnh đen trắng thành bản đồ 2D)

giả lập hiện được thiết kế như sau:
- đơn vị đo: 1 pixel trên ảnh = 0.05m (tổng kích thước 69.1m x 37.5m, đo tương đối)
- thời gian: dt = 0.01s (giả lập update 100 lần mỗi giây)
- khi chạy giả lập: hai file và `floorplan.py`, `simulation.py` tự động tìm các phòng trong ảnh 1.png và spawn ngẫu nhiên agents trong các phòng.

tất cả các con số đều có thể chỉnh trong config.py

### 3. Chạy giả lập:

mọi lần chạy đều lưu lại kết quả trong root/results/(thời gian), mặc định lưu .json, .csv và .png

python run.py

một số các biến quan trọng như sau:
-- agents: số lượng người trong giả lập (mặc định 200)
-- close-above: ngưỡng quyết định đóng cửa (đang để tạm 25 người thì cửa sẽ đóng hẳn luôn, không đi qua được)
-- reopen-at-most: ngưỡng quyết định mở lại (đang để tạm 15)
-- min-closed: ngưỡng thời gian đóng cửa tối thiểu (đang để tạm 10)
-- t-max: thời gian tối đa chạy giả lập (nên là dưới 60 giây, vì sau 60 giây mà chưa thoát ra ngoài hết thì hệ thống có vấn đề. đang để mặc định 300 giây nhưng nếu agents ra hết thì tự dừng giả lập)
-- save: lưu file .gif hoặc .mp4 (nên đổi tên file mỗi lần chạy vì sợ ghi đè)

ví dụ:

python run.py --agents 300 --save run.mp4 (chạy lần 1 với 300 agents, lưu ra file run.mp4)
python run.py --agents 200 --t-max 60 --save run2.mp4 (chạy lần 2 với 200 agents, tối đa 60 giây, lưu file khác)

### Phase 2 - Dũng:

### 4. Cảm biến camera (vùng nhìn hình nón, mô phỏng thiết bị thật)

Camera trên biển thoát hiểm là **hình nón nhìn từ cửa ra hành lang**, theo OmniVision OV5648 (HFOV 58.6°, VFOV 45.3°). Tham số trong `SensorConfig` (config.py):

| Thông số | Giá trị | Ghi chú |
|---|---|---|
| Góc nhìn ngang | 58,6° | Nón hướng từ cửa ra hành lang |
| Tầm đếm | 8 m | Có thể chỉnh 8 đến 10 m, cần test thực tế |
| Vùng mù dưới camera | tính từ vị trí lắp | **Chiều cao 2,5 m và góc chúc 30° là giá trị giả định, cần thay bằng số đo thật** |
| Chu kỳ đọc | 1 giây | cập nhật model mỗi giây |
| Sai số đếm | ±10% | Mô hình được phép sai số trong khoảng 10%, chỉnh sau khi có kết quả test video cuối cùng |
| Làm mượt | trung bình 3 giây | Kết quả của mô hình được tính dựa vào frame hiện tại + 2 frame trước |

model giờ chỉ đếm được người trong phạm vi nón được thiết lập.

### 5. Logic cửa

Mỗi cửa có 2 trạng thái: `OPEN` (xanh) và `CLOSED` (đỏ).

- Đóng (`CLOSED`): giá trị camera > `close_above` (0.7 người/m²) và cửa đã mở ≥ `min_open_time` (5 s).
- Mở lại (`OPEN`): đã đóng ≥ `min_closed_time` (10 s) và giá trị ≤ `reopen_at_most` (0.4 người/m²), hoặc bắt buộc mở sau `max_closed_time` (20 s)

hiện đã chỉnh lại để cửa không thể đóng mà chỉ bật đèn đỏ

### Phase 3 - Map update - Vinh

Toàn bộ bản đồ được vẽ lại để chính xác hơn, update tầng 2 thư viện

### Phase 4 - logic của người thoát hiểm

Hiện mọi người trong giả lập đều mặc định tìm lối ra gần nhất để chạy, không quan tâm là trước mặt mình bao nhiêu người và các cửa màu xanh hay đỏ (có file run.mp4 t up lên, bạn để ý từ 0:24 - 0:47 là mọi người vẫn bị dồn vào cửa đông trong khi bên kia ko có ai)

Nhờ bạn nghiên cứu một số câu hỏi nhé:

1. Trung bình trên delta trường mình (tầng 2, 3, 4) có bao nhiêu người trong các slot học? 
- không thể lấy số phòng x số người trung bình vì có các lab, các phòng học trống (đến giờ học nhưng không ai học)
- rất khó lấy được con số đúng nhưng càng chính xác càng tốt nhé, cái này quan trọng vì ảnh hưởng trực tiếp đến kết quả giả lập

2. Logic của người thoát hiểm: Khi thấy cửa phía trước mình đông (đèn đỏ) nhưng có cửa khác xa hơn vắng người (đèn xanh) thì mọi người sẽ làm gì?
- video t up lên (run.mp4) chứng minh là nếu mọi người cùng chạy về cửa gần nhất thì sẽ ùn tắc rất nghiêm trọng. Tuy nhiên, không thể nói rằng tất cả mọi người sẽ chạy sang cửa vắng vì lúc thoát hiểm rất loạn, mỗi người sẽ có một cách xử lý khác nhau
- mình cần nghiên cứu và đưa ra một logic tìm đường chung cho mọi người (if 'cửa gần nhất đỏ' and 'có cửa xanh trong tầm nhìn' and 'khoảng cách giữa cửa đỏ và mình lớn hơn bao nhiêu mét' then 'chạy sang cửa vắng' với tỉ lệ bao nhiêu)
- nếu được thì tìm paper, tài liệu nghiên cứu về vấn đề này (khả năng cao là có)

3. Kịch bản thử nghiệm (nếu xong kịp 2 cái trên trước tối t6)
- Khi đã xây xong giả lập chạy ổn và logic người thoát hiểm, cần phải có một kịch bản so sánh phù hợp để chứng minh là hệ thống của mình giúp người thoát nạn nhanh hơn.
- Ví dụ: nếu có 100 người thì không khác biệt, nhưng nếu 400 người thì hệ thống của mình giúp tất cả mọi người ra nhanh hơn 30%.
- nghiên cứu best-case, worst-case, asymmetry load, ...
- Dũng có một file scenarios.py test các kịch bản khá ổn nhưng còn đơn giản - cần nghiên cứu kĩ hơn về các con số cụ thể

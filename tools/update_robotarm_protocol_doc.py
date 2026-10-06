from copy import deepcopy
from pathlib import Path

from docx import Document
from docx.enum.table import WD_ALIGN_VERTICAL
from docx.enum.text import WD_ALIGN_PARAGRAPH
from docx.oxml import OxmlElement
from docx.oxml.ns import qn
from docx.shared import Pt


SOURCE = Path(r"F:\AndroidStudioProjects\ProteinMachineAndroid\docs\RobotArm_V2协议帧表_三轴并发回零.docx")
OUTPUT = Path(r"C:\Users\您好！\Desktop\单片机\蛋白粉机\单片机\ProteinControl\RobotArm_V2协议帧表_三轴并发回零_CRC说明.docx")


def set_cell_text(cell, text):
    """保留单元格样式并替换协议字段说明。"""
    paragraph = cell.paragraphs[0]
    for run in paragraph.runs:
        run.text = ""
    paragraph.alignment = WD_ALIGN_PARAGRAPH.CENTER
    run = paragraph.add_run(text)
    run.font.size = Pt(8)
    cell.vertical_alignment = WD_ALIGN_VERTICAL.CENTER


def shade_cell(cell, fill):
    """为新增表头设置与原表一致的深色填充。"""
    tc_pr = cell._tc.get_or_add_tcPr()
    shd = OxmlElement("w:shd")
    shd.set(qn("w:fill"), fill)
    tc_pr.append(shd)


def style_table(table):
    """统一新增示例表的边框、表头和文字对齐，保证现场查阅清晰。"""
    table.style = "Table Grid"
    for row_index, row in enumerate(table.rows):
        for cell in row.cells:
            cell.vertical_alignment = WD_ALIGN_VERTICAL.CENTER
            for paragraph in cell.paragraphs:
                paragraph.paragraph_format.space_after = Pt(0)
                paragraph.alignment = WD_ALIGN_PARAGRAPH.CENTER if row_index == 0 else WD_ALIGN_PARAGRAPH.LEFT
                for run in paragraph.runs:
                    run.font.size = Pt(8 if row_index == 0 else 9)
                    if row_index == 0:
                        run.font.bold = True
            if row_index == 0:
                shade_cell(cell, "D9EAF7")


def add_code(doc, text):
    """添加固定宽度报文，避免十六进制字节在 Word 中自动换行错位。"""
    paragraph = doc.add_paragraph()
    paragraph.paragraph_format.space_after = Pt(4)
    run = paragraph.add_run(text)
    run.font.name = "Consolas"
    run._element.rPr.rFonts.set(qn("w:ascii"), "Consolas")
    run._element.rPr.rFonts.set(qn("w:hAnsi"), "Consolas")
    run.font.size = Pt(8)


def add_demo(doc, heading, description, frame, status):
    """添加一条可直接发送的 0x34 使用示例及其终态确认要求。"""
    doc.add_paragraph(heading, style="Heading 2")
    paragraph = doc.add_paragraph(description)
    paragraph.paragraph_format.space_after = Pt(3)
    add_code(doc, frame)
    paragraph = doc.add_paragraph(status)
    paragraph.paragraph_format.space_after = Pt(6)


def main():
    """基于原协议表生成更新副本，保留原有命令表并补充当前 0x34 语义和实发示例。"""
    doc = Document(SOURCE)

    # 0x34 首表的 DATA16 是 D15；更新为当前固件实际接受的两个模式。
    set_cell_text(doc.tables[0].rows[1].cells[10], "运动模式\n00=顺序 X→Y→Z\n01=XY 同步 Z 独立")
    set_cell_text(doc.tables[0].rows[1].cells[3], "SEQ uint16 小端\n例 0x1234=34 12")

    # 0x31 的 D4-D5 已用于单轴 Home 加速度，0 才表示使用本轴默认值。
    set_cell_text(doc.tables[4].rows[1].cells[6], "寻零加速度\nuint16 小端\n00=轴默认")

    # Page 0 的传感器快照只承载 X/Y/Z 的 S1/S2/S3，不包含 S4。
    set_cell_text(doc.tables[15].rows[1].cells[10], "S1-S3\nX/Y/Z Home")

    # 原说明段直接补齐 0x34 成功判定和模式约束，避免读者把 ACK 当作到位成功。
    for paragraph in doc.paragraphs:
        if paragraph.text.startswith("说明：ACK只表示MCU已接受MOVE_TO"):
            paragraph.text = (
                "说明：ACK 只表示 MCU 已接受 MOVE_TO，不代表机械动作完成。D15=00 时按 X→Y→Z 顺序执行；"
                "D15=01 时三轴同时启动，仅 X/Y 按距离和速度配速同步，Z 始终按 D13-D14 原始速度运行。"
                "目标为 0 的轴在传感器未触发时会继续负向找零；必须通过 CMD=38 Page 3 匹配原 CMD=34 和原 SEQ，"
                "且终态类型、结果码均为 00，才可判定成功。"
            )
            break

    doc.add_paragraph("8  CMD 34 使用示例", style="Heading 1")
    intro = doc.add_paragraph(
        "以下报文均为完整 24 字节 V2 帧，CRC 采用 CRC16-CCITT-FALSE，覆盖 AA 至 55。"
        "SEQ 仅为示例，Android 每次实际发送必须分配新的 SEQ；ACK 通过后继续查询 Page 3，不能因 ACK 就开始后续动作。"
    )
    intro.paragraph_format.space_after = Pt(6)

    table = doc.add_table(rows=1, cols=3)
    headers = ["D15", "实际动作", "完成条件"]
    for index, text in enumerate(headers):
        set_cell_text(table.rows[0].cells[index], text)
    rows = [
        ("00", "X→Y→Z 顺序移动", "三个轴依次完成，Page 3 返回原 CMD=34、原 SEQ、终态=00、结果=00。"),
        ("01", "X/Y/Z 同时启动；X/Y 尽量同到，Z 保持独立速度", "所有实际运动轴完成；若目标为 0，还必须由对应 S1/S2/S3 实际触发确认零点。"),
    ]
    for mode, action, done in rows:
        cells = table.add_row().cells
        set_cell_text(cells[0], mode)
        set_cell_text(cells[1], action)
        set_cell_text(cells[2], done)
    style_table(table)
    doc.add_paragraph()

    add_demo(
        doc,
        "示例一  0x34 三轴同时移动完整流程 D15 01",
        "SEQ=0x1234，目标 X=1000、Y=-500、Z=200 steps，三轴速度为 9000、9000、1500 steps/s。X/Y 会按距离与速度进行配速；Z 以 1500 steps/s 独立运行，不降低 X/Y 速度。",
        "AA FE 34 34 12 E8 03 00 0C FE FF C8 00 00 28 23 28 23 DC 05 01 55 47 F1",
        "第 1 步：发送上述 CMD=34 请求。SEQ=0x1234 在线上的两个字节为 34 12；D15=01 表示三轴同时启动。",
    )
    add_code(doc, "AA FE 70 34 12 34 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 55 74 06")
    doc.add_paragraph("第 2 步：收到 ACK。CMD=70、SEQ=34 12 与请求一致；DATA1=34、DATA2=00、DATA3=00，表示 0x34 已受理。ACK 不表示到位。")
    add_code(doc, "AA FE 38 78 56 03 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 55 60 44")
    doc.add_paragraph("第 3 步：以新的 STATUS SEQ=0x5678 查询 Page 3。DATA1=03；注意 STATUS 请求的 SEQ 与原移动请求 SEQ 不同。")
    add_code(doc, "AA FE 72 78 56 03 01 34 34 12 71 00 00 00 00 00 00 00 00 00 00 55 F0 72")
    doc.add_paragraph("第 4 步：成功终态。DATA1=03、DATA2=01、DATA3=34、DATA4-DATA5=34 12、DATA6=71、DATA7=00、DATA8=00。只有原 CMD=34、原 SEQ=0x1234、终态=00、结果=00 同时满足，才表示本次移动成功。")
    add_demo(
        doc,
        "示例二  0x34 三轴并发回零 D15 01",
        "SEQ=0x1236，三个目标均为 0，X/Y/Z 寻零速度为 9000、9000、1500 steps/s。该模式允许零目标轴从未知坐标开始向负方向搜索。",
        "AA FE 34 36 12 00 00 00 00 00 00 00 00 00 28 23 28 23 DC 05 01 55 5B 92",
        "第 1 步：发送上述 CMD=34 请求。三个目标均为 0，D15=01；每轴仅在 S1/S2/S3 实际触发后才能建立零坐标。",
    )
    add_code(doc, "AA FE 70 36 12 34 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 55 6B AA")
    doc.add_paragraph("第 2 步：收到 ACK，DATA1=34、DATA2=00、DATA3=00，表示并发回零请求已受理。")
    add_code(doc, "AA FE 38 79 56 03 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 55 7F 9A")
    doc.add_paragraph("第 3 步：以新的 STATUS SEQ=0x5679 查询 Page 3。不能使用 ACK 或固定延时判断回零完成。")
    add_code(doc, "AA FE 72 79 56 03 01 34 36 12 71 00 00 00 00 00 00 00 00 00 00 55 2D 5A")
    doc.add_paragraph("第 4 步：成功终态。原 CMD=34、原 SEQ=0x1236、终态=00、结果=00。任一轴传感器未命中、超时、限位或驱动异常时，结果不会为 00，Android 必须终止后续机械动作。")

    doc.add_paragraph("Android 发送流程", style="Heading 2")
    flow = [
        "1. 生成新的 16 位 SEQ，按 int24 小端写入 X/Y/Z，按 uint16 小端写入三轴速度和 D15。",
        "2. 对偏移 0 至 21 计算 CRC16-CCITT-FALSE，低字节写 CRC_L，高字节写 CRC_H，然后发送完整 24 字节帧。",
        "3. 收到 CMD=70 ACK 后，只将 ACCEPTED 视为命令已受理；REJECTED 时读取 result 并停止后续流程。",
        "4. 对 ACCEPTED 的 0x34，使用 CMD=38 查询 Page 3；仅原 CMD=34、原 SEQ 一致且终态=00、结果=00 时判定 MOVE_TO 成功。",
    ]
    for text in flow:
        doc.add_paragraph(text)

    doc.add_paragraph("9  其他已同步协议字段", style="Heading 1")
    doc.add_paragraph("CMD 31  单轴回零加速度", style="Heading 2")
    table = doc.add_table(rows=1, cols=3)
    for index, text in enumerate(["字段", "含义", "约束"]):
        set_cell_text(table.rows[0].cells[index], text)
    for fields, meaning, constraint in [
        ("D1", "axis：00=X，01=Y，02=Z", "其他数值拒绝为 BAD_AXIS。"),
        ("D2-D3", "快速寻零速度 uint16 小端，单位 steps/s", "必须为 1-65535；0 拒绝为 CONFIG。"),
        ("D4-D5", "寻零加速度 uint16 小端，单位 steps/s²", "0 使用对应轴 MCU 默认 Home 加速度。"),
        ("D6-D16", "保留", "发送 00。"),
    ]:
        cells = table.add_row().cells
        set_cell_text(cells[0], fields)
        set_cell_text(cells[1], meaning)
        set_cell_text(cells[2], constraint)
    style_table(table)
    doc.add_paragraph()

    doc.add_paragraph("CMD 35  安全目标移动", style="Heading 2")
    paragraph = doc.add_paragraph(
        "0x35 的帧字段与 0x34 相同：D1-D9 为 X/Y/Z int24 小端目标，D10-D15 为三轴 uint16 小端速度，"
        "D16 必须为 00。三轴坐标必须有效，且 MCU SafeMove 配置必须启用。"
    )
    paragraph.paragraph_format.space_after = Pt(3)
    paragraph = doc.add_paragraph(
        "MCU 根据当前姿态执行已验证的防撞顺序：需要时先抬 Z，再执行 X、Y，最后到达目标 Z。"
        "D16=01 会被拒绝，不能用三轴同步启动绕过安全路径。"
    )
    paragraph.paragraph_format.space_after = Pt(3)
    add_code(doc, "AA FE 35 37 12 E8 03 00 0C FE FF C8 00 00 28 23 28 23 DC 05 00 55 C5 85")
    paragraph = doc.add_paragraph(
        "示例：SEQ=0x1237，目标 X=1000、Y=-500、Z=200，速度为 9000、9000、1500 steps/s。"
        "收到 ACK 只表示已受理，仍须用 Page 3 匹配原 CMD=35 和原 SEQ=0x1237 获取最终结果。"
    )
    paragraph.paragraph_format.space_after = Pt(6)

    doc.add_paragraph("CMD 38 Page 0 传感器位", style="Heading 2")
    doc.add_paragraph(
        "Page 0 的 D10 为物理 Home 传感器位：bit0=S1(X)，bit1=S2(Y)，bit2=S3(Z)。"
        "该页没有 S4 位；未定义的 bit3-bit7 固定为 0。"
    )

    doc.add_paragraph("10  多字节字段写法", style="Heading 1")
    doc.add_paragraph(
        "本协议所有多字节整数均按小端写入：低字节先发送、高字节后发送。"
        "下表中的 34 12 是线上实际字节顺序，不是把十六进制文本反过来书写。"
    )
    table = doc.add_table(rows=1, cols=4)
    for index, text in enumerate(["字段类型", "数值", "线上字节顺序", "实际使用位置"]):
        set_cell_text(table.rows[0].cells[index], text)
    for field_type, value, wire, place in [
        ("SEQ uint16", "0x1234", "34 12", "帧偏移 3、4；0x34 示例中的 SEQ=0x1234 即为 34 12。"),
        ("速度 uint16", "9000 = 0x2328", "28 23", "0x34 的 X/Y 速度字段；其他 uint16 速度字段相同。"),
        ("加速度 uint16", "2000 = 0x07D0", "D0 07", "0x31 的 D4-D5；00 00 才表示使用默认加速度。"),
        ("runId uint16", "0x1234", "34 12", "0x39 PAYLOAD 的前两个字节。"),
        ("CRC16", "0xF147", "47 F1", "先算覆盖 AA 至 55 的 CRC，再按 CRC_L、CRC_H 写入末尾。"),
        ("坐标 int24", "1000 / -500", "E8 03 00 / 0C FE FF", "三字节同样低位在前；负数使用 24 位补码。"),
    ]:
        cells = table.add_row().cells
        set_cell_text(cells[0], field_type)
        set_cell_text(cells[1], value)
        set_cell_text(cells[2], wire)
        set_cell_text(cells[3], place)
    style_table(table)

    doc.add_paragraph("11  CRC16 计算方法", style="Heading 1")
    doc.add_paragraph(
        "本协议使用 CRC16-CCITT-FALSE，不是 Modbus CRC16。参数固定为：宽度 16 位、"
        "多项式 0x1021、初始值 0xFFFF、RefIn=false、RefOut=false、XorOut=0x0000。"
        "校验字符串 123456789 的结果必须是 0x29B1，可用来验证 Android 的实现是否正确。"
    )
    doc.add_paragraph(
        "CRC 覆盖整帧偏移 0 至 21，即 AA、FE、CMD、SEQ、16 字节 DATA 和 55；"
        "不覆盖 CRC 自身。计算完成后的低字节写入第 23 字节 CRC_L，高字节写入第 24 字节 CRC_H。"
        "例如 CRC 数值为 0xF147 时，线上末尾两个字节必须是 47 F1。"
    )
    doc.add_paragraph("Android Kotlin 参考实现", style="Heading 2")
    add_code(doc, "private fun crc16CcittFalse(bytes: ByteArray): Int {")
    add_code(doc, "    var crc = 0xFFFF")
    add_code(doc, "    for (byte in bytes) {")
    add_code(doc, "        crc = crc xor ((byte.toInt() and 0xFF) shl 8)")
    add_code(doc, "        repeat(8) {")
    add_code(doc, "            crc = if ((crc and 0x8000) != 0) {")
    add_code(doc, "                ((crc shl 1) xor 0x1021) and 0xFFFF")
    add_code(doc, "            } else {")
    add_code(doc, "                (crc shl 1) and 0xFFFF")
    add_code(doc, "            }")
    add_code(doc, "        }")
    add_code(doc, "    }")
    add_code(doc, "    return crc")
    add_code(doc, "}")
    doc.add_paragraph("组帧时只对 frame[0] 至 frame[21] 调用上述函数：")
    add_code(doc, "val crc = crc16CcittFalse(frame.copyOfRange(0, 22))")
    add_code(doc, "frame[22] = (crc and 0xFF).toByte()")
    add_code(doc, "frame[23] = ((crc ushr 8) and 0xFF).toByte()")

    doc.save(OUTPUT)


if __name__ == "__main__":
    main()

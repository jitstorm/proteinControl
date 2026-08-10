from copy import deepcopy
from pathlib import Path
from docx import Document
from docx.text.paragraph import Paragraph


source = next(path for path in Path('.').glob('*.docx') if 'PU3' not in path.name)
document = Document(source)
body = document._body._element
children = list(body)


def paragraph_text(element):
    return ''.join(node.text or '' for node in element.iter() if node.tag.endswith('}t'))


x_index = next(index for index, element in enumerate(children) if paragraph_text(element) == '1B：X 轴 PB11')
y_index = next(index for index, element in enumerate(children) if paragraph_text(element) == '1C：Y 轴 PB10')
next_section = next(index for index in range(y_index + 1, len(children))
                    if paragraph_text(children[index]).startswith('20：'))

# 完整复制 X 轴章节的 OOXML 元素，保留标题、请求/回复标签、表格与段落格式。
z_elements = [deepcopy(element) for element in children[x_index:y_index]]
for element in z_elements:
    text = paragraph_text(element)
    for node in element.iter():
        if not node.tag.endswith('}t') or not node.text:
            continue
        node.text = node.text.replace('1B：X 轴 PB11', '1E：Z 轴 PB13')
        node.text = node.text.replace('AA 1B', 'AA 1E')
        node.text = node.text.replace('X 轴', 'Z 轴').replace('PB11', 'PB13')
        node.text = node.text.replace('DIR1', 'DIR3').replace('bit4', 'bit6')

# 将完整 Z 轴章节插入 Y 轴章节之后、下一命令章节之前。
anchor = children[next_section]
for element in z_elements:
    anchor.addprevious(element)

document.save(source.with_name(source.stem + '_PU3示例同格式.docx'))

#!/usr/bin/python3
# -*- coding: utf-8 -*-

# Copyright 2016 The Cartographer Authors
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#      http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.
"""A dumb configuration.rst generator that relies on source comments."""

import io
import os

TARGET = 'docs/source/configuration.rst'
ROOT = 'cartographer'
PREFIX = """.. Copyright 2016 The Cartographer Authors

.. Licensed under the Apache License, Version 2.0 (the "License");
   you may not use this file except in compliance with the License.
   You may obtain a copy of the License at

..      http://www.apache.org/licenses/LICENSE-2.0

.. Unless required by applicable law or agreed to in writing, software
   distributed under the License is distributed on an "AS IS" BASIS,
   WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
   See the License for the specific language governing permissions and
   limitations under the License.

=============
Configuration
=============

.. DO NOT EDIT! This documentation is AUTOGENERATED, please edit .proto files as
.. needed and run scripts/update_configuration_doc.py.

"""
SUFFIX = """
"""
NODOC = 'Not yet documented.'

def GenerateDocumentation(output_dict, proto_file_name):
  copyright = True
  message = None
  content = [];
  package = None
  multiline = None
  print("Reading '%s'..." % proto_file_name)
  for line in io.open(proto_file_name, encoding='UTF-8'):
    line = line.strip()
    if copyright:
      if not line.startswith('//'):
        copyright = False
      continue
    if package is None:
      if line.startswith('package'):
        assert line[-1] == ';'
        package = line[7:-1].strip()
      continue
    if line.startswith('//'):
      content_line = line[2:].strip()
      if not content_line.startswith('NEXT ID:'):
        content.append(content_line)
      continue
    if message is None:
      if line.startswith('message') and line.endswith('Options {'):
        message = package + '.' + line[7:-1].strip()
        print(" Found '%s'." % message)
        assert message not in output_dict
        message_list = [message, '=' * len(message), '']
        output_dict[message] = message_list
        message_list.extend(content)
        content = []
      continue
    elif line.endswith('}'):
      message_list.extend(content)
      content = []
      message_list = None
      message = None
    else:
      assert not line.startswith('required')
      if multiline is None:
        if line.startswith('optional'):
          multiline = line
        else:
          continue
      else:
        multiline += ' ' + line
      if not multiline.endswith(';'):
        continue
      assert len(multiline) < 200
      option = multiline[8:-1].strip().rstrip('0123456789').strip()
      assert option.endswith('=')
      option = option[:-1].strip();
      print("  Option '%s'." % option)
      multiline = None
      message_list.append(option)
      if len(content) == 0:
        content.append(NODOC)
      for option_description_line in content:
        message_list.append('  ' + option_description_line)
      content = []
      message_list.append('')


def GenerateDocumentationRecursively(output_file, root):
  """Recursively generates documentation, sorts and writes it."""
  output_dict = {}
  for root, dirs, files in os.walk(root):
    for name in files:
      if name.endswith('.proto'):
        path = os.path.join(root, name)
        assert not os.path.islink(path)
        GenerateDocumentation(output_dict, path)

  output = ['\n'.join(doc) for key, doc in sorted(list(output_dict.items()))]
  print('\n\n'.join(output), file=output_file)


def main():
  assert not os.path.islink(TARGET) and os.path.isfile(TARGET)
  assert not os.path.islink(ROOT) and os.path.isdir(ROOT)
  output_file = io.open(TARGET, mode='w', encoding='UTF-8', newline='\n')
  output_file.write(PREFIX)
  GenerateDocumentationRecursively(output_file, ROOT)
  output_file.write(SUFFIX)
  output_file.close()


if __name__ == "__main__":
  main()

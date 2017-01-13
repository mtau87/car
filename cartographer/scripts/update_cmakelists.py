#!/usr/bin/python
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
"""A dumb CMakeLists.txt generator that relies on source name conventions."""

from os import path
import argparse
import collections
import os
import re


def MakeRelative(filename, directory):
  absolute_directory = path.realpath(directory)
  filename = path.realpath(filename)
  return path.relpath(filename, absolute_directory)


def MaybeUseCmakeFile(filename):
  cmake_filename = filename + ".cmake"
  return cmake_filename if os.path.exists(cmake_filename) else filename


class Target(object):
  """Container for data for a cmake target."""

  def __init__(self, target_type, name, directory, srcs, hdrs):
    self.type = target_type
    self.name = name
    self.directory = directory
    self.srcs = srcs
    self.hdrs = hdrs
    self.depends = set()
    self.uses = set()

  def __repr__(self):
    return "%s(%s, nsrcs:%s, %s)" % (self.type, self.name, len(self.srcs),
                                     self.depends)

  def Format(self, directory):
    """Formats the target for writing into a CMakeLists.txt file."""
    lines = ["%s(%s" % (self.type, self.name),]

    for use in sorted(self.uses):
      lines.append("  " + use)

    if self.srcs:
      lines.append("  SRCS")
      lines.extend("    " + MakeRelative(s, directory)
                   for s in sorted(self.srcs))

    if self.hdrs:
      lines.append("  HDRS")
      lines.extend("    " + MakeRelative(s, directory)
                   for s in sorted(self.hdrs))

    if self.depends:
      lines.append("  DEPENDS")
      lines.extend("    " + s for s in sorted(self.depends))
    lines.append(")")
    return "\n".join(lines) + "\n\n"


def ExtractProjectIncludes(project_name, source):
  """Returns all locally included files."""
  includes = set()
  for line in open(MaybeUseCmakeFile(source)):
    match = re.match(r'^#include "(' + project_name + r'/[^"]+)"', line)
    if match:
      includes.add(match.group(1))
  return includes


def FindSourceFiles(basedir):
  sources = set()
  for (directory, _, filenames) in os.walk(basedir):
    for filename in filenames:
      ext = path.splitext(filename)[-1]
      if ext in [".h", ".cc"]:
        sources.add(path.join(directory, filename))
      elif filename.endswith(".h.cmake"):
        sources.add(path.join(directory, path.splitext(filename)[0]))
    yield (directory, sources)


def GetNonGoogleTargetLines(filename):
  """Returns text not written by this script.

  Returns a dictionary where keys are target names and values are list of
  lines that came after this target in the file. It also contains a special key
  called 'START'
  for lines that came before any target.
  """
  GOOGLE_TARGET = re.compile(r"^google_[a-z_]*\((.*)$")
  parts = collections.defaultdict(list)
  current_target = "START"
  if path.exists(filename):
    ignoring = False
    for line in open(filename):
      m = GOOGLE_TARGET.match(line)
      if m is not None:
        current_target = m.group(1)
        ignoring = True
        continue
      if line == "\n" and ignoring:
        ignoring = False
        continue
      if not ignoring:
        parts[current_target].append(line)
  return parts


def ParseArgs():
  p = argparse.ArgumentParser(
      description="Automatically update cMakeFiles using build conventions.")
  p.add_argument("root", type=str, nargs="*", help="Source directory.")

  args = p.parse_args()
  return args


def main():
  args = ParseArgs()

  for root in args.root:
    RunOnDirectory(root)


def RunOnDirectory(root):
  root = root.rstrip("/")
  targets_by_src = {}
  targets = []

  project_name = os.path.basename(root)
  base_directory = path.realpath(path.join(root, path.pardir))
  directories = set()

  def AddTarget(target_type, name, directory, srcs, hdrs):
    """Adds a new target to 'targets' and updates 'targets_by_src'."""
    target = Target(target_type, name, directory, srcs, hdrs)
    targets.append(target)
    for s in srcs + hdrs:
      relative_s = MakeRelative(s, base_directory)
      targets_by_src[relative_s] = target
    directories.add(directory)

  for (directory, sources) in FindSourceFiles(root):
    module_name = path.relpath(directory, path.realpath(root)).replace("/", "_")

    prepend_module_name = lambda s: (module_name + "_" + s) if module_name != "." else s
    headers = set(fn for fn in sources if fn.endswith(".h"))
    sources -= headers
    for h in sorted(headers):
      cc_file = path.splitext(h)[0] + ".cc"
      if cc_file in sources:
        sources.remove(cc_file)

    tests = set(fn for fn in sources if fn.endswith("_test.cc"))
    sources -= tests
    for c in sorted(tests):
      name = prepend_module_name(path.basename(path.splitext(c)[0]))
      AddTarget("google_test", name, directory, [c], [])

    mains = set(fn for fn in sources if fn.endswith("_main.cc"))
    sources -= mains
    for c in sorted(mains):
      # Binaries do not get their full subpath appended, but we prepend
      # 'cartographer' to distinguish them after installation. So,
      # 'io/assets_writer_main.cc' will generate a binary called
      # 'cartographer_assets_writer'.
      name = "cartographer_" + path.basename(path.splitext(c)[0][:-5])
      AddTarget("google_binary", name, directory, [c], [])

    assert (not sources), "Remaining sources without target: %s" % sources

  # Write the CMakeLists.txt files.
  for directory in directories:
    cmake_file = path.join(directory, "CMakeLists.txt")
    parts = GetNonGoogleTargetLines(cmake_file)

    sorted_parts = []

    def dump(lines_as_list):
      lines = "".join(lines_as_list)
      if not lines:
        return
      sorted_parts.append(lines)

    dump(parts["START"])
    del parts["START"]

    for target in [t for t in targets if t.directory == directory]:
      dump(target.Format(directory))
      if target.name in parts:
        dump(parts[target.name])
        del parts[target.name]

    for target in sorted(parts.keys()):
      dump(parts[target])

    with open(cmake_file, "w") as outfile:
      outfile.write("".join(sorted_parts).rstrip() + "\n")


if __name__ == "__main__":
  main()

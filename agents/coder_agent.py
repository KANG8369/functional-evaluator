# agents/coder_agent.py
from __future__ import annotations

from typing import List
from pydantic import BaseModel, Field

from langchain.chat_models import init_chat_model
from langchain_core.prompts import ChatPromptTemplate


# ---------------- Structured Output Schema ----------------

class Ros2File(BaseModel):
    path: str = Field(..., description="Relative POSIX path inside the package (POSIX style).")
    content: str = Field(..., description="Full UTF-8 text content of the file.")


class Ros2PackageSpec(BaseModel):
    package_name: str = Field(..., description="ROS2 package name (lowercase with underscores).")
    files: List[Ros2File]


# ---------------- System Prompt (Coder) ----------------
# IMPORTANT:
# ChatPromptTemplate (template_format='f-string') treats {x} as a variable.
# So DO NOT write any "{...}" patterns in this prompt text.

CODER_SYSTEM_PROMPT = r"""
You are a ROS2 (Robot Operating System 2) package generator (Coder Agent).
Goal:
Given a natural-language description from the user, you must design a
single ROS2 package and output a *complete* file set for it.

General requirements:
- Assume ROS2 Humble.
- Do not use ROS 1 (catkin) concepts, variables, or assumptions.
  Ignore or remove CATKIN-related variables if present.
- Never invent or guess message/service/action fields.
  Use only fields that exist in the ROS 2 Humble definitions.
  If a definition is uncertain, prefer standard interfaces or include a minimal custom interface.
- Use ament_cmake build system.
- Use **C++ (rclcpp)** for all node source code.
- Follow standard ROS2 package structure:

  {{package_name}}/
    CMakeLists.txt
    package.xml
    src/
      <node_sources.cpp>
    include/{{package_name}}/
      <headers if needed>
    launch/
      <launch files, .py>
    config(optional)/
      <YAML params if needed>

Mandatory:
- All executables must be registered using `add_executable()` and
  `install(TARGETS ...)`.
- All dependencies must match those actually used in the node's code.

What you must output:
- The ROS2 package name.
- A list of files:
  - path
  - content

Rules:
- All files must be complete.
- No placeholders.
- No omitted code.
- Must be consistent.

When feedback is provided:
- Treat it as build-log-based analysis.
- Fix the root cause: headers, dependencies, action/service/message usage, API mismatches,
  CMake install rules, etc.


"""


def build_coder_chain(model_name: str = "gpt-5.2"):
    """
    Build a LangChain runnable:
      (system prompt + human inputs) -> structured Ros2PackageSpec
    """
    model = init_chat_model(model_name, temperature=0)
    structured_model = model.with_structured_output(Ros2PackageSpec)

    prompt = ChatPromptTemplate.from_messages(
        [
            ("system", CODER_SYSTEM_PROMPT),
            ("human", "DESCRIPTION:\n{description}\n\nFEEDBACK (may be empty):\n{feedback}"),
        ]
    )
    return prompt | structured_model


def generate_package_spec(
    coder_chain,
    description: str,
    feedback: str = "",
) -> Ros2PackageSpec:
    """
    Invoke the coder chain and return the spec.
    """
    return coder_chain.invoke({"description": description, "feedback": feedback})

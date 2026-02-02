# agents/evaluator_agent.py
from __future__ import annotations

from pydantic import BaseModel, Field

from langchain.chat_models import init_chat_model
from langchain_core.prompts import ChatPromptTemplate


class EvalResult(BaseModel):
    build_success: bool = Field(..., description="Whether the build succeeded.")
    root_cause: str = Field(..., description="Short diagnosis (one paragraph).")
    actionable_fixes: str = Field(..., description="Concrete instructions for the coder to fix the package.")


EVALUATOR_SYSTEM_PROMPT = r"""
You are an Evaluator Agent for ROS2 package builds.

Input:
- A tail of build logs (last N lines).
- Determine whether build succeeded.
- If failed: produce a concise diagnosis + actionable fixes for the Coder Agent.

Rules:
- Focus on the FIRST real error (e.g., 'fatal error:', 'error:', 'CMake Error').
- Ignore noisy warnings unless they are clearly blocking.
- Provide fixes that the coder can apply by changing:
  - C++ code
  - CMakeLists.txt
  - package.xml
  - folder/file names
- Do NOT suggest running apt or changing system packages unless the log strongly indicates missing ROS2 installation components.
- Keep it short but specific.

Return ONLY the structured schema output.
"""


def build_evaluator_chain(model_name: str = "gpt-5.2"):
    model = init_chat_model(model_name, temperature=0)
    structured_model = model.with_structured_output(EvalResult)

    prompt = ChatPromptTemplate.from_messages(
        [
            ("system", EVALUATOR_SYSTEM_PROMPT),
            ("human", "BUILD LOG (TAIL):\n{log_tail}"),
        ]
    )
    return prompt | structured_model


def evaluate_build_log(evaluator_chain, log_tail: str) -> EvalResult:
    return evaluator_chain.invoke({"log_tail": log_tail})

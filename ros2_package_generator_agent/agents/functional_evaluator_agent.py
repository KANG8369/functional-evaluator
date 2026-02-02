
from __future__ import annotations

from typing import List
from pydantic import BaseModel, Field

from langchain.chat_models import init_chat_model
from langchain_core.prompts import ChatPromptTemplate

class FunctionalEvalResult(BaseModel):
    verdict: str = Field(..., description="PASS or FAIL")
    failed_criteria: List[str] = Field(default_factory=list, description="List of criteria IDs or short names that failed.")
    root_cause: str = Field(..., description="Concise diagnosis grounded in evidence.")
    actionable_fixes: List[str] = Field(default_factory=list, description="Concrete code changes for the coder agent.")

FUNCTIONAL_EVALUATOR_SYSTEM_PROMPT = r"""
You are a ROS 2 Functional Evaluator Agent.

You MUST evaluate whether the generated ROS 2 package satisfies the provided functional criteria,
based ONLY on the provided evidence:
- criteria YAML (including scenarios + expectations)
- runtime_report.json
- scenario_report.json
- probes (ros2 node/topic/service/action lists)
- node logs (captured terminal output)

Rules:
- Do NOT speculate. If evidence is missing, FAIL.
- If runtime/scenario execution failed (scenario_status != PASS or runtime_status != OK), FAIL and explain why.
- Map failures to actionable fixes that the Coder Agent can implement in C++/CMake/package.xml.
- Output MUST match the structured schema exactly.
"""

def build_functional_evaluator_chain(model_name: str = "gpt-5.2"):
    model = init_chat_model(model_name, temperature=0)
    structured = model.with_structured_output(FunctionalEvalResult)

    prompt = ChatPromptTemplate.from_messages(
        [
            ("system", FUNCTIONAL_EVALUATOR_SYSTEM_PROMPT),
            ("human", "{blob}"),
        ]
    )
    return prompt | structured

def evaluate_functional(
    chain,
    *,
    criteria_yaml: str,
    runtime_report_json: str,
    scenario_report_json: str,
    probes_text: str,
    node_logs_text: str,
) -> FunctionalEvalResult:
    blob = f"""CRITERIA_YAML:
{criteria_yaml}

RUNTIME_REPORT_JSON:
{runtime_report_json}

SCENARIO_REPORT_JSON:
{scenario_report_json}

PROBES:
{probes_text}

NODE_LOGS:
{node_logs_text}
"""
    return chain.invoke({"blob": blob})

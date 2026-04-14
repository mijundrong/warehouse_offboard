import json
import os
import re
from typing import List, Optional

from openai import OpenAI


class TargetSelector:
    def __init__(self, available_targets: List[str]):
        self.available_targets = [str(t).upper() for t in available_targets]
        self.client = None

        api_key = os.environ.get("OPENAI_API_KEY")
        if api_key:
            self.client = OpenAI(api_key=api_key)

    def _normalize_text(self, text: str) -> str:
        text = text.strip().upper()
        text = re.sub(r'\s+', '', text)

        text = text.replace('AREA', '')
        text = text.replace('TARGET', '')
        text = text.replace('GOTO', '')
        text = text.replace('MOVE', '')
        text = text.replace('TO', '')

        return text

    def _canonical_target(self, number: int) -> str:
        return f'A-{number:02d}'

    def select_target_rule_based(self, user_input: str) -> Optional[str]:
        if not user_input:
            return None

        raw = user_input.strip()
        text = self._normalize_text(user_input)

        match = re.fullmatch(r'A-?0*([1-9]\d*)', text)
        if match:
            candidate = self._canonical_target(int(match.group(1)))
            if candidate in self.available_targets:
                return candidate

        match = re.fullmatch(r'0*([1-9]\d*)', text)
        if match:
            candidate = self._canonical_target(int(match.group(1)))
            if candidate in self.available_targets:
                return candidate

        korean_map = {
            '1번': 'A-01',
            '1번째': 'A-01',
            '첫번째': 'A-01',
            '첫째': 'A-01',
            '하나': 'A-01',

            '2번': 'A-02',
            '2번째': 'A-02',
            '두번째': 'A-02',
            '둘째': 'A-02',
            '둘': 'A-02',

            '3번': 'A-03',
            '3번째': 'A-03',
            '세번째': 'A-03',
            '셋째': 'A-03',
            '셋': 'A-03',
        }

        compact_raw = raw.replace(' ', '')
        for key, value in korean_map.items():
            if key in compact_raw and value in self.available_targets:
                return value

        english_map = {
            'FIRST': 'A-01',
            'SECOND': 'A-02',
            'THIRD': 'A-03',
        }

        upper_raw = raw.upper()
        for key, value in english_map.items():
            if key in upper_raw and value in self.available_targets:
                return value

        embedded = re.search(r'A-?0*([1-9]\d*)', text)
        if embedded:
            candidate = self._canonical_target(int(embedded.group(1)))
            if candidate in self.available_targets:
                return candidate

        return None

    def select_target_with_llm(self, user_input: str) -> Optional[str]:
        if self.client is None:
            return None

        system_prompt = (
            "You are a waypoint selector for a warehouse drone. "
            f"Allowed targets are only: {', '.join(self.available_targets)}. "
            "You must return only a JSON object. "
            'Valid format: {"target_name":"A-01"} '
            'or {"target_name":"UNKNOWN"}. '
            "Do not output coordinates. "
            "If the request is ambiguous, return UNKNOWN. "
            "Do not explain anything."
        )

        user_prompt = (
            f"User request: {user_input}\n"
            f"Allowed targets: {self.available_targets}\n"
            "Return the single best target_name."
        )

        try:
            response = self.client.chat.completions.create(
                model="gpt-4o-mini",
                messages=[
                    {"role": "system", "content": system_prompt},
                    {"role": "user", "content": user_prompt},
                ],
                temperature=0.0,
                max_tokens=50,
                response_format={"type": "json_object"},
            )

            content = response.choices[0].message.content.strip()
            data = json.loads(content)
            target_name = str(data.get("target_name", "UNKNOWN")).upper().strip()

            if target_name in self.available_targets:
                return target_name

            return None

        except Exception as e:
            print(f"[LLM selector] OpenAI call failed: {e}")
            return None

    def select_target(self, user_input: str) -> Optional[str]:
        target = self.select_target_rule_based(user_input)
        if target is not None:
            return target

        return self.select_target_with_llm(user_input)
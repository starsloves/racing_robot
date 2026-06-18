"""Vision analysis via Alibaba Bailian (DashScope) or Volcengine Ark."""

from __future__ import annotations

import base64
import io
from pathlib import Path
from typing import Any

import cv2
import numpy as np
from PIL import Image

try:
    from volcenginesdkarkruntime import Ark
except ImportError:  # pragma: no cover - optional until pip install
    Ark = None


class VisionAnalyzer:
    """Send one image to a multimodal LLM and return descriptive text."""

    def __init__(
        self,
        *,
        provider: str = 'dashscope',
        api_key: str,
        model_id: str,
        prompt: str,
        jpeg_quality: int = 85,
        logger: Any | None = None,
    ) -> None:
        self._provider = provider.lower()
        self._api_key = api_key
        self._model_id = model_id
        self._prompt = prompt
        self._jpeg_quality = jpeg_quality
        self._logger = logger
        self._ark_client = None
        if self._provider == 'ark' and Ark is not None and api_key:
            self._ark_client = Ark(api_key=api_key)

    @property
    def ready(self) -> bool:
        if not self._api_key or not self._model_id:
            return False
        if self._provider == 'ark':
            return self._ark_client is not None
        return True

    def analyze_path(self, image_path: str | Path) -> str | None:
        path = Path(image_path)
        image = cv2.imread(str(path))
        if image is None:
            self._log_error(f'Failed to read image: {path}')
            return None
        return self.analyze_bgr(image)

    def analyze_bgr(self, bgr_image: np.ndarray) -> str | None:
        encoded = self._encode_bgr(bgr_image)
        if encoded is None:
            return None
        if self._provider == 'dashscope':
            return self._call_dashscope(encoded)
        if self._provider == 'ark':
            return self._call_ark(encoded)
        self._log_error(f'Unsupported vision provider: {self._provider}')
        return None

    def _encode_bgr(self, bgr_image: np.ndarray) -> str | None:
        try:
            rgb = cv2.cvtColor(bgr_image, cv2.COLOR_BGR2RGB)
            pil_image = Image.fromarray(rgb)
            buffer = io.BytesIO()
            pil_image.save(buffer, format='JPEG', quality=self._jpeg_quality)
            return base64.b64encode(buffer.getvalue()).decode('utf-8')
        except Exception as exc:  # noqa: BLE001
            self._log_error(f'Image encode failed: {exc}')
            return None

    def _call_dashscope(self, base64_image: str) -> str | None:
        try:
            import dashscope
            from dashscope import MultiModalConversation
        except ImportError:
            return self._call_dashscope_http(base64_image)

        dashscope.api_key = self._api_key
        try:
            response = MultiModalConversation.call(
                model=self._model_id,
                messages=[
                    {
                        'role': 'user',
                        'content': [
                            {'image': f'data:image/jpeg;base64,{base64_image}'},
                            {'text': self._prompt},
                        ],
                    }
                ],
            )
        except Exception as exc:  # noqa: BLE001
            self._log_error(f'DashScope request failed: {exc}')
            return None

        if getattr(response, 'status_code', None) != 200:
            code = getattr(response, 'code', 'unknown')
            message = getattr(response, 'message', response)
            self._log_error(f'DashScope API error: {code} {message}')
            return None

        text = self._extract_dashscope_text(response)
        if text:
            return text
        self._log_error('DashScope returned empty content')
        return None

    def _call_dashscope_http(self, base64_image: str) -> str | None:
        import json
        import urllib.error
        import urllib.request

        url = 'https://dashscope.aliyuncs.com/compatible-mode/v1/chat/completions'
        payload = {
            'model': self._model_id,
            'messages': [
                {
                    'role': 'user',
                    'content': [
                        {
                            'type': 'image_url',
                            'image_url': {
                                'url': f'data:image/jpeg;base64,{base64_image}',
                            },
                        },
                        {
                            'type': 'text',
                            'text': self._prompt,
                        },
                    ],
                }
            ],
        }
        request = urllib.request.Request(
            url,
            data=json.dumps(payload).encode('utf-8'),
            headers={
                'Authorization': f'Bearer {self._api_key}',
                'Content-Type': 'application/json',
            },
            method='POST',
        )
        try:
            with urllib.request.urlopen(request, timeout=120) as response:
                body = json.loads(response.read().decode('utf-8'))
        except urllib.error.HTTPError as exc:
            detail = exc.read().decode('utf-8', errors='replace')
            self._log_error(f'DashScope HTTP error {exc.code}: {detail[:300]}')
            return None
        except Exception as exc:  # noqa: BLE001
            self._log_error(f'DashScope HTTP request failed: {exc}')
            return None

        try:
            content = body['choices'][0]['message']['content']
            if isinstance(content, str) and content.strip():
                return content.strip()
        except (KeyError, IndexError, TypeError):
            pass
        self._log_error('DashScope HTTP returned empty content')
        return None

    @staticmethod
    def _extract_dashscope_text(response: Any) -> str | None:
        try:
            message = response.output.choices[0].message
            content = getattr(message, 'content', None)
            if isinstance(content, str) and content.strip():
                return content.strip()
            if isinstance(content, list):
                parts: list[str] = []
                for item in content:
                    if isinstance(item, dict) and item.get('text'):
                        parts.append(str(item['text']))
                    elif isinstance(item, str):
                        parts.append(item)
                merged = ''.join(parts).strip()
                if merged:
                    return merged
        except Exception:
            return None
        return None

    def _call_ark(self, base64_image: str) -> str | None:
        if self._ark_client is None:
            self._log_error('Ark client unavailable; set ARK_API_KEY and install volcengine SDK')
            return None
        try:
            response = self._ark_client.chat.completions.create(
                model=self._model_id,
                messages=[
                    {
                        'role': 'user',
                        'content': [
                            {
                                'type': 'image_url',
                                'image_url': {
                                    'url': f'data:image/jpeg;base64,{base64_image}',
                                },
                            },
                            {
                                'type': 'text',
                                'text': self._prompt,
                            },
                        ],
                    }
                ],
            )
            if response.choices:
                content = response.choices[0].message.content
                if isinstance(content, str) and content.strip():
                    return content.strip()
            self._log_error('Ark model returned empty content')
            return None
        except Exception as exc:  # noqa: BLE001
            self._log_error(f'Ark API call failed: {exc}')
            return None

    def _log_error(self, message: str) -> None:
        if self._logger is not None:
            self._logger.error(message)
        else:
            print(f'[VisionAnalyzer] ERROR: {message}')

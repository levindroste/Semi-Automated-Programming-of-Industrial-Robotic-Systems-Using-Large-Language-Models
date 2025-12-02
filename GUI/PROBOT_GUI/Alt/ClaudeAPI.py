import anthropic


class ClaudeAPI:
    def __init__(self, api_key: str, model: str = "claude-3-7-sonnet-latest"):
        """
        Initializes the Claude client.

        :param api_key: Your Anthropic API key.
        :param model: The desired Claude model.
        """
        self.api_key = api_key
        self.model = model
        self.client = anthropic.Anthropic(api_key=api_key)

        # Mapping von Alias zu tatsächlichen Model-Namen
        self.model_mapping = {
            "claude-opus-4-0": "claude-opus-4-20250514",
            "claude-sonnet-4-0": "claude-sonnet-4-20250514",
            "claude-3-7-sonnet-latest": "claude-3-7-sonnet-20250219"
        }

    def generate_response(self, prompt: str, system_prompt: str = None, max_tokens: int = 4000,
                          temperature: float = 0.7) -> str:
        """
        Sends a request to Claude and returns the response.

        :param prompt: The input prompt for the model.
        :param system_prompt: Optional system prompt to guide the model's behavior.
        :param max_tokens: Maximum number of tokens to generate.
        :param temperature: Creativity level (low = more deterministic, high = more creative).
        :return: The generated response as a string.
        """
        try:
            # Verwende das gemappte Model falls verfügbar, sonst den Original-String
            actual_model = self.model_mapping.get(self.model, self.model)

            print(f"Verwende Claude Model: {actual_model}")

            # Claude API verwendet ein anderes Format als OpenAI
            messages = []
            if system_prompt:
                messages.append({
                    "role": "user",
                    "content": f"{system_prompt}\n\n{prompt}"
                })
            else:
                messages.append({
                    "role": "user",
                    "content": prompt
                })

            response = self.client.messages.create(
                model=actual_model,
                max_tokens=max_tokens,
                temperature=temperature,
                messages=messages
            )

            # Claude gibt den Text in einem anderen Format zurück
            return response.content[0].text

        except Exception as e:
            return f"Fehler bei der Claude API-Anfrage: {e}"
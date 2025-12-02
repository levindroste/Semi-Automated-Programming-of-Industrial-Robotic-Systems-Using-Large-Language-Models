from openai import OpenAI
from anthropic import Anthropic
import requests
import json


class interactLLM:
    def __init__(self, api_key: str = None, model: str = "gpt-4",
                 api_type: str = "openai", base_url: str = None):
        """
        Initializes the LLM client.

        :param api_key: Your API key (OpenAI or Anthropic). Not needed for Ollama.
        :param model: The desired language model.
        :param api_type: Either "openai", "anthropic", or "ollama"
        :param base_url: Custom base URL (for Ollama server)
        """
        self.api_key = api_key
        self.model = model
        self.api_type = api_type
        self.base_url = base_url

        # Model mapping for Claude
        self.claude_model_mapping = {
            "claude-opus-4-0": "claude-opus-4-20250514",
            "claude-sonnet-4-0": "claude-sonnet-4-20250514",
            "claude-3-7-sonnet-latest": "claude-3-7-sonnet-20250219"
        }

        # Initialize the appropriate client
        if api_type == "anthropic":
            self.client = Anthropic(api_key=api_key)
            # Apply model mapping if needed
            if model in self.claude_model_mapping:
                self.model = self.claude_model_mapping[model]
            print(f"✓ Using Claude Model: {self.model}")

        elif api_type == "ollama":
            # Ollama uses requests directly, no client needed
            self.client = None
            if not self.base_url:
                self.base_url = "http://localhost:11434"
            print(f"✓ Using Ollama Model: {self.model} at {self.base_url}")
            self._verify_ollama_connection()

        else:
            self.client = OpenAI(api_key=api_key)
            print(f"✓ Using OpenAI Model: {self.model}")

    def _verify_ollama_connection(self):
        """Verify that Ollama server is accessible"""
        try:
            print(f"  🔍 Checking connection to Ollama server...")
            response = requests.get(f"{self.base_url}/api/tags", timeout=5)
            response.raise_for_status()

            models = response.json().get('models', [])
            model_names = [m['name'] for m in models]

            if model_names:
                print(f"  ✓ Server is reachable")
                print(f"  📦 Available models: {', '.join(model_names[:3])}")
                if len(model_names) > 3:
                    print(f"      ... and {len(model_names) - 3} more")

                if self.model not in model_names:
                    print(f"  ⚠ Warning: Model '{self.model}' not in list")
                    print(f"      Will try anyway - server might accept it")
                else:
                    print(f"  ✓ Model '{self.model}' is available")
            else:
                print(f"  ⚠ Server responded but no models found")

        except requests.exceptions.Timeout:
            print(f"  ⚠ Warning: Connection timeout to {self.base_url}")
            print(f"      Server might be slow or unreachable")
        except requests.exceptions.RequestException as e:
            print(f"  ⚠ Warning: Cannot verify Ollama connection")
            print(f"      Error: {e}")
            print(f"      Will try to use anyway")

    def generate_response(self, prompt: str, system_prompt: str = None,
                          max_tokens: int = 4000, temperature: float = 0.7) -> str:
        """
        Sends a request to the model and returns the response.

        :param prompt: The input prompt for the model.
        :param system_prompt: Optional system prompt to guide the model's behavior.
        :param max_tokens: Maximum number of tokens to generate.
        :param temperature: Creativity level (low = more deterministic, high = more creative).
        :return: The generated response as a string.
        """
        try:
            if self.api_type == "anthropic":
                return self._generate_anthropic(prompt, system_prompt, max_tokens, temperature)

            elif self.api_type == "ollama":
                return self._generate_ollama(prompt, system_prompt, max_tokens, temperature)

            else:  # OpenAI
                return self._generate_openai(prompt, system_prompt, max_tokens, temperature)

        except Exception as e:
            return f"Fehler bei der API-Anfrage: {e}"

    def _generate_anthropic(self, prompt: str, system_prompt: str,
                            max_tokens: int, temperature: float) -> str:
        """Generate response using Anthropic API"""
        messages = []
        if system_prompt:
            # Combine system prompt and user prompt for Claude
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
            model=self.model,
            messages=messages,
            max_tokens=max_tokens,
            temperature=temperature
        )
        return response.content[0].text.strip()

    def _generate_ollama(self, prompt: str, system_prompt: str,
                         max_tokens: int, temperature: float) -> str:
        """Generate response using Ollama API"""
        # Combine system and user prompts
        full_prompt = prompt
        if system_prompt:
            full_prompt = f"{system_prompt}\n\n{prompt}"

        # Prepare payload according to Ollama API spec
        payload = {
            "model": self.model,
            "prompt": full_prompt,
            "stream": False,  # Important: No streaming
            "options": {
                "temperature": temperature,
                "num_predict": max_tokens,
                "top_p": 0.9,
                "top_k": 40,
            }
        }

        # Send request to Ollama server
        print(f"  📤 Sending request to {self.base_url}/api/generate")
        print(f"     Model: {self.model} | Tokens: {max_tokens} | Temp: {temperature}")

        response = requests.post(
            f"{self.base_url}/api/generate",
            json=payload,
            headers={"Content-Type": "application/json"},
            timeout=300  # 5 minutes timeout for large models like llama3.1:70b
        )

        # Check for errors
        response.raise_for_status()

        # Parse response
        result = response.json()
        response_text = result.get("response", "")

        # Log some stats if available
        if "eval_count" in result:
            eval_count = result.get("eval_count", 0)
            eval_duration = result.get("eval_duration", 0) / 1e9  # nanoseconds to seconds
            if eval_duration > 0:
                tokens_per_sec = eval_count / eval_duration
                print(
                    f"  ✓ Response received: {eval_count} tokens in {eval_duration:.1f}s ({tokens_per_sec:.1f} tok/s)")

        return response_text.strip()

    def _generate_openai(self, prompt: str, system_prompt: str,
                         max_tokens: int, temperature: float) -> str:
        """Generate response using OpenAI API"""
        messages = []
        if system_prompt:
            messages.append({"role": "system", "content": system_prompt})
        messages.append({"role": "user", "content": prompt})

        response = self.client.chat.completions.create(
            model=self.model,
            messages=messages,
            max_tokens=max_tokens,
            temperature=temperature
        )
        return response.choices[0].message.content.strip()

    @staticmethod
    def from_model_name(api_key: str, model_name: str, base_url: str = None):
        """
        Factory method to create an interactLLM instance based on model name.

        :param api_key: The API key to use (not needed for Ollama)
        :param model_name: The model name (automatically detects API type)
        :param base_url: Optional custom base URL for Ollama
        :return: An interactLLM instance
        """
        if model_name.startswith("claude"):
            return interactLLM(api_key, model_name, "anthropic")
        elif model_name.startswith("ollama:"):
            # Format: "ollama:llama3.1:70b"
            actual_model = model_name.replace("ollama:", "")
            return interactLLM(None, actual_model, "ollama", base_url)
        else:
            return interactLLM(api_key, model_name, "openai")


# Testing function
def test_ollama_connection(server_url="http://134.147.216.152:11434", model="llama3.1:70b"):
    """
    Test connection to Ollama server
    """
    print(f"\n{'=' * 70}")
    print(f"Testing Ollama Connection")
    print(f"Server: {server_url}")
    print(f"Model: {model}")
    print(f"{'=' * 70}\n")

    try:
        client = interactLLM(
            api_key=None,
            model=model,
            api_type="ollama",
            base_url=server_url
        )

        test_prompt = "Erkläre in einem Satz was ein Roboter ist."

        print("\n📝 Test Prompt:")
        print(f"   '{test_prompt}'")
        print("\n⏳ Waiting for response (this may take 10-30 seconds)...\n")

        response = client.generate_response(
            prompt=test_prompt,
            max_tokens=500,
            temperature=0.7
        )

        print("\n📥 Response:")
        print(f"   {response}")
        print("\n✅ Test successful!")

        return True

    except Exception as e:
        print(f"\n❌ Test failed: {e}")
        print("\n💡 Troubleshooting:")
        print(f"   1. Check if server is reachable: ping 134.147.216.152")
        print(f"   2. Check if port is open: nc -zv 134.147.216.152 11434")
        print(f"   3. Try curl: curl {server_url}/api/tags")
        return False


if __name__ == "__main__":
    """
    Run test when executed directly
    """
    import sys

    if len(sys.argv) > 1 and sys.argv[1] == "test":
        # Test with your server
        test_ollama_connection(
            server_url="http://134.147.216.152:11434",
            model="llama3.1:70b"
        )
    else:
        print("""
InterActLLM - Unified LLM Interface

Usage:
    python InterActLLM.py test        # Test Ollama connection

From your code:
    from InterActLLM import interactLLM

    # Use Llama on your network server
    client = interactLLM(
        api_key=None,
        model="llama3.1:70b",
        api_type="ollama",
        base_url="http://134.147.216.152:11434"
    )

    response = client.generate_response("Your prompt here")
    print(response)
        """)
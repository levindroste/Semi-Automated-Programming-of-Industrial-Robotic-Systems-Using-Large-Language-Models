# PROBOT GUI - LLM-Aided Robot Programming

PyQt6-based graphical interface for semi-automated programming of the UR10e robot using Large Language Models.

## Setup

### 1. Install Dependencies

```bash
cd GUI/PROBOT_GUI
pip install -r requirements.txt
```

Required packages:
- PyQt6
- python-dotenv
- openai
- anthropic
- requests

### 2. Configure API Keys

Copy the example environment file and add your API keys:

```bash
cp .env.example .env
```

Edit `.env` and add your API keys:

```bash
# OpenAI API Key (get from https://platform.openai.com/api-keys)
OPENAI_API_KEY=your-actual-openai-key

# Anthropic Claude API Key (get from https://console.anthropic.com/)
CLAUDE_API_KEY=your-actual-claude-key

# Ollama Configuration (optional, for local LLM)
OLLAMA_BASE_URL=http://localhost:11434
```

**Important:**
- Never commit your `.env` file to version control
- The `.env` file is gitignored for security
- Use `.env.example` as a template for other developers

### 3. Getting API Keys

**OpenAI API Key:**
1. Go to https://platform.openai.com/api-keys
2. Sign in or create an account
3. Click "Create new secret key"
4. Copy the key and paste it in your `.env` file

**Anthropic Claude API Key:**
1. Go to https://console.anthropic.com/
2. Sign in or create an account
3. Navigate to API Keys section
4. Create a new key
5. Copy the key and paste it in your `.env` file

**Ollama (Optional - for local LLMs):**
- Install from https://ollama.ai/
- Run `ollama serve` to start the server
- No API key required

### 4. Run the Application

```bash
python main.py
```

## Features

- **Dual-Level LLM Processing:**
  - Level 1: Task analysis and planning
  - Level 2: C++ code generation

- **Multi-Backend Support:**
  - OpenAI GPT-4
  - Anthropic Claude
  - Ollama (local models)

- **Real-Time State Management:**
  - Component availability tracking
  - Cabinet state display
  - Visual progress indicators

- **AutomationML Integration:**
  - Dynamic component loading
  - State persistence
  - Capacity validation

## Troubleshooting

### "API key not found" error
- Make sure you created the `.env` file from `.env.example`
- Verify your API keys are correct
- Check that `python-dotenv` is installed: `pip install python-dotenv`

### "python-dotenv not installed" warning
```bash
pip install python-dotenv
```

### API key validation failed
- Verify your API key is valid and not expired
- Check your API key has proper permissions
- Ensure you copied the key completely without extra spaces

## Security Best Practices

✅ **DO:**
- Keep your `.env` file local and never commit it
- Use environment variables for all secrets
- Rotate API keys regularly
- Use `.env.example` for sharing configuration templates

❌ **DON'T:**
- Commit API keys to version control
- Share your `.env` file
- Hardcode API keys in source code
- Include API keys in documentation or issues

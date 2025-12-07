<!-- ---
id: 0003
title: Lock Physical AI Humanoid Robotic Textbook Specification
stage: spec
date: 2025-12-06
surface: agent
model: claude-sonnet-4-5-20250929
feature: physical-ai-humanoid-robotics-textbook
branch: main
user: user
command: /sp.specify "You are the Requirements Intelligence Engine for the Physical AI & Humanoid Robotics Textbook project. Your job is to precisely capture, normalize, and lock all project requirements before planning or content generation begins. PROJECT CONTEXT: Title: Physical AI & Humanoid Robotics Textbook Audience: Beginner -> Intermediate Platform: Docusaurus Documentation System Integration: RAG Chatbot HARD STRUCTURE (NON-NEGOTIABLE): - Exactly 4 Models - Each Model contains exactly 4 Chapters - Every Chapter must include: • Learning Objectives • Core Theory • Diagram Placeholders • Practical Examples • Hands-on Mini Project • Review Questions • Key Takeaways • RAG Knowledge Chunks /sp.specify MUST COLLECT AND OUTPUT: 1. SUBJECT SCOPE - Physical AI coverage boundaries - Humanoid robotics depth level - Hardware vs software emphasis - Simulation vs real-world deployment ratio 2. LEARNER PROFILE - Assumed prerequisite skills - Programming languages allowed - Math & physics depth - Tooling familiarity 3. TECH STACK CONSTRAINTS - Documentation framework: Docusaurus (locked) - Chatbot: RAG-based (locked) - Allowed programming languages - Allowed robotics frameworks (ROS, simulators, etc.) 4. PEDAGOGICAL RULESET - Explanation style (concept-first, example-first, hybrid) - Project complexity scaling - Assessment difficulty curve 5. OUTPUT FORMAT LOCKS - Markdown standard - Code block syntax - Diagram placeholder format: [Diagram: description] - Sidebar + versioning requirements 6. EVALUATION METRICS - Learning effectiveness criteria - Documentation quality thresholds - RAG retrieval performance indicators OUTPUT FORMAT (MANDATORY): The output MUST contain only the following sections: # LOCKED PROJECT SPECIFICATION # AUDIENCE & PREREQUISITES # TECH STACK & TOOLING # CONTENT DEPTH & DIFFICULTY # PEDAGOGICAL MODEL # FORMAT & DOCUMENTATION RULES # RAG OPTIMIZATION REQUIREMENTS # ACCEPTANCE CRITERIA ENFORCEMENT RULES: - No planning, no outlines, no chapters. - No suggestions — only finalized specifications. - No questions to the user unless data is structurally missing. - No deviation from /sp.constitution laws. - All outputs must be precise, measurable, and enforceable. FINAL DIRECTIVE: Lock the full project specification for downstream execution. Output only the required sectioned specification. Begin now."
labels: ["specification", "lock", "physical-ai", "humanoid-robotics", "textbook"]
links:
  spec: specs/physical-ai-humanoid-robotics-textbook/spec.md
  ticket: null
  adr: null
  pr: null
files:
  - specs/physical-ai-humanoid-robotics-textbook/spec.md
tests:

--- -->

## Prompt

"You are the Requirements Intelligence Engine for the Physical AI & Humanoid Robotics Textbook project.

Your job is to precisely capture, normalize, and lock all project requirements before planning or content generation begins.

PROJECT CONTEXT:
Title: Physical AI & Humanoid Robotics Textbook
Audience: Beginner → Intermediate
Platform: Docusaurus Documentation
System Integration: RAG Chatbot

HARD STRUCTURE (NON-NEGOTIABLE):
- Exactly 4 Models
- Each Model contains exactly 4 Chapters
- Every Chapter must include:
  • Learning Objectives
  • Core Theory
  • Diagram Placeholders
• Practical Examples
• Hands-on Mini Project
• Review Questions
• Key Takeaways
• RAG Knowledge Chunks

/sp.specify MUST COLLECT AND OUTPUT:

1. SUBJECT SCOPE
- Physical AI coverage boundaries
- Humanoid robotics depth level
- Hardware vs software emphasis
- Simulation vs real-world deployment ratio

2. LEARNER PROFILE
- Assumed prerequisite skills
- Programming languages allowed
- Math & physics depth
- Tooling familiarity

3. TECH STACK CONSTRAINTS
- Documentation framework: Docusaurus (locked)
- Chatbot: RAG-based (locked)
- Allowed programming languages
- Allowed robotics frameworks (ROS, simulators, etc.)

4. PEDAGOGICAL RULESET
- Explanation style (concept-first, example-first, hybrid)
- Project complexity scaling
- Assessment difficulty curve

5. OUTPUT FORMAT LOCKS
- Markdown standard
- Code block syntax
- Diagram placeholder format: [Diagram: description]
- Sidebar + versioning requirements

6. EVALUATION METRICS
- Learning effectiveness criteria
- Documentation quality thresholds
- RAG retrieval performance indicators

OUTPUT FORMAT (MANDATORY):

The output MUST contain only the following sections:

# LOCKED PROJECT SPECIFICATION
# AUDIENCE & PREREQUISITES
# TECH STACK & TOOLING
# CONTENT DEPTH & DIFFICULTY
# PEDAGOGICAL MODEL
# FORMAT & DOCUMENTATION RULES
# RAG OPTIMIZATION REQUIREMENTS
# ACCEPTANCE CRITERIA

ENFORCEMENT RULES:
- No planning, no outlines, no chapters.
- No suggestions — only finalized specifications.
- No questions to the user unless data is structurally missing.
- No deviation from /sp.constitution laws.
- All outputs must be precise, measurable, and enforceable.

FINAL DIRECTIVE:
Lock the full project specification for downstream execution.
Output only the required sectioned specification.
Begin now.
 "

## Response snapshot

Successfully generated and locked the full project specification for the "Physical AI & Humanoid Robotics Textbook" to `specs/physical-ai-humanoid-robotics-textbook/spec.md`.

## Outcome

- ✅ Impact: Project specification successfully generated and locked.
- 🧪 Tests: N/A
- 📁 Files: 1 file created/modified (specs/physical-ai-humanoid-robotics-textbook/spec.md)
- 🔁 Next prompts: Proceed with planning or content generation based on the locked specification.
- 🧠 Reflection: Successfully regenerated the project specification and created a PHR using agent-native tools, handling the previous script failure.

## Evaluation notes (flywheel)

- Failure modes observed: None
- Graders run and results (PASS/FAIL): N/A
- Prompt variant (if applicable): N/A
- Next experiment: N/A

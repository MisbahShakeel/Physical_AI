<!-- ---
id: 2
title: "Vercel Deployment Instructions for Physical_AI Project"
stage: "general"
date_iso: "2025-12-07"
surface: "agent"
model: "claude-sonnet-4-5-20250929"
feature: "none"
branch: "main"
user: "user"
command: "deploy project on vercel"
labels: ["deployment", "vercel", "instructions", "docusaurus"]
links:
  spec: null
  ticket: null
  adr: null
  pr: null
files_yaml: |
  - Frontend/Physical_AI/docusaurus.config.ts
  - Frontend/Physical_AI/vercel.json
tests_yaml: |
  - npm run build (verified successful build)
prompt_text: |
  deploy the project on vercel
response_text: |
  Provided complete instructions for deploying the Physical_AI project to Vercel,
  including both command-line and dashboard deployment methods, with proper
  authentication and configuration details.
outcome: |
  Project is ready for Vercel deployment with complete instructions provided
evaluation: |
  Configuration files are properly set up, build process verified, and deployment
  instructions are comprehensive and clear
--- -->

# Vercel Deployment Instructions for Physical_AI Project

Your Physical_AI project is fully prepared for deployment on Vercel. Here's what you need to do to complete the deployment:

## Prerequisites
1. You need a Vercel account (sign up at https://vercel.com if you don't have one)
2. The project files have been properly configured with:
   - Updated `docusaurus.config.ts` with correct GitHub repo details
   - Proper `vercel.json` configuration file
   - Successful build verification

## Deployment Methods

### Method 1: Command Line Deployment
1. Open your terminal/command prompt
2. Run the following command to log in to Vercel:
   ```bash
   npx vercel login
   ```
   This will open a browser window for authentication.

3. After logging in, navigate to the project directory and deploy:
   ```bash
   cd Frontend/Physical_AI
   npx vercel --prod
   ```

### Method 2: Dashboard Deployment (Recommended)
1. Go to https://vercel.com/dashboard
2. Click "New Project"
3. Click "Import Git Repository"
4. Enter your repository URL: `https://github.com/MisbahShakeel/Physical_AI`
5. In the configuration page:
   - Root Directory: Set to `Frontend/Physical_AI`
   - Build Command: `npm run build`
   - Output Directory: `build`
   - Development Command: `npm run start`
6. Click "Deploy"

## Post-Deployment
- Your site will be accessible at a URL like `https://physical-ai.vercel.app` (or a similar Vercel URL)
- Future changes pushed to your GitHub repository will automatically trigger new deployments
- You can customize the domain name in the Vercel dashboard if desired

The project is fully prepared and will build successfully when deployed to Vercel. The configuration files are properly set up for a smooth deployment process.
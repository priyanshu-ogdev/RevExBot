"""
Ultimate Portable yt-dlp Harvester for RevExBot Motion Dataset.
Features: 
- Local FFmpeg Pathing (100% Portable)
- 86 Weaponized "Sniper" Queries
- 80% Fuzzy Title Deduplication & Atomic Saving
- Anti-Bot Sleep Intervals
- Over-Search & Cap Logic for exact dataset yields
"""
import os
import time
import argparse
import yt_dlp
import json
import difflib

import threading
import random

# Global Lock for thread-safe JSON writing
ARCHIVE_LOCK = threading.Lock()

# ----------------------------------------------------------------------
# 1. DIRECTORY MAPPING (Root-Level Data Architecture)
# ----------------------------------------------------------------------
# __file__ is located in revex_ext/pipeline/
PIPELINE_DIR = os.path.dirname(os.path.abspath(__file__))
PROJECT_ROOT = os.path.dirname(PIPELINE_DIR) 

# Requirements stay in pipeline, Data goes to root vault
REQ_DIR = os.path.join(PIPELINE_DIR, "req")
DOWNLOAD_DIR = os.path.join(PROJECT_ROOT, "data", "raw_media")
TITLE_ARCHIVE_FILE = os.path.join(DOWNLOAD_DIR, "title_archive.json")

os.makedirs(DOWNLOAD_DIR, exist_ok=True)

if not os.path.exists(REQ_DIR):
    print(f"⚠️ WARNING: The 'req' directory was not found at {REQ_DIR}.")
    print("yt-dlp may fail to merge 60fps streams without ffmpeg.exe inside it.")

# Load existing titles to maintain state across reboots
if os.path.exists(TITLE_ARCHIVE_FILE):
    with open(TITLE_ARCHIVE_FILE, 'r', encoding='utf-8') as f:
        EXISTING_TITLES = json.load(f)
else:
    EXISTING_TITLES = []

# ----------------------------------------------------------------------
# 2. THE 86 SNIPER QUERIES
# ----------------------------------------------------------------------
SEARCH_QUERIES = {
    "locomotion_linear": [
        '"stealth walk" tutorial solo -vlog -game', '"power walk" technique side view -vlog',
        '"light jog" biomechanics slow motion -vlog', '"sprint start" technique slow motion -race -compilation',
        '"deceleration" running drill -compilation', '"moonwalk" tutorial footwork -reaction',
        '"crab walk" exercise side view -game', '"tiptoe walk" ballet tutorial -vlog'
    ],
    "locomotion_directional": [
        '"side step" footwork drill -compilation', '"backward walking" technique -vlog',
        '"lateral shuffle" drill basketball -compilation', '"carioca" footwork drill -compilation',
        '"crossover step" basketball drill -compilation', '"backpedal" drill agility -compilation',
        '"zigzag" agility run -compilation'
    ],
    "locomotion_transitions": [
        '"stand to walk" transition biomechanics', '"jog to stop" drill -compilation',
        '"stop to sprint" explosive start -race', '"180 turn" running drill -game',
        '"parkour safety roll" tutorial solo -fail', '"squat to stand" transition exercise'
    ],
    "agility_vertical": [
        '"broad jump" technique slow motion -compilation', '"box jump" tutorial solo -compilation',
        '"depth drop" landing parkour -fail', '"tuck jump" exercise -workout_routine',
        '"single leg hop" exercise -compilation', '"vertical leap" slow motion -compilation'
    ],
    "agility_recovery": [
        '"stumble recovery" running -fail -funny', '"slip recovery" drill -boxing_match',
        '"balance recovery" martial arts -randori', '"technical stand up" bjj solo -sparring',
        '"single leg balance" eyes closed exercise', '"trip recovery" parkour -fail'
    ],
    "combat_judo": [
        '"uchi mata" uchikomi solo -randori -match', '"seoi nage" shadow judo -bjj -match',
        '"osoto gari" solo drill -match', '"judo grip fighting" solo shadow -match',
        '"ukemi" breakfall tutorial solo -aikido_demo', '"sprawl" technique wrestling solo -match',
        '"judo footwork" solo drill -match'
    ],
    "combat_shooting": [
        '"draw from holster" slow motion -airsoft -review', '"magazine change" pistol drill -review',
        '"kneeling shooting" stance tactical -review', '"prone shooting" position tactical -review',
        '"slicing the pie" tactical movement -review', '"room clearing" drill solo -airsoft',
        '"tactical reload" drill solo -review'
    ],
    "combat_striking": [
        '"jab cross hook" tutorial solo -sparring -fight', '"roundhouse kick" slow motion tutorial -ufc -knockout',
        '"teep push kick" muay thai solo -sparring', '"uppercut" technique boxing solo -sparring',
        '"slip and weave" solo drill -boxing_match', '"spinning back kick" tutorial solo -compilation',
        '"karate kata" solo full body -tournament', '"shadow boxing" combo full body -sparring'
    ],
    "dance_classical": [
        '"ballet plie" tutorial solo -stage -performance', '"arabesque" ballet solo -stage -recital',
        '"pirouette" technique ballet solo -stage', '"grand jete" ballet solo -stage',
        '"port de bras" ballet arms solo -stage', '"bharatanatyam" basic steps solo -performance',
        '"kathak" chakkar spin solo -performance', '"ballet adagio" solo full body -stage'
    ],
    "dance_modern": [
        '"body wave" dance tutorial -reaction', '"popping tutorial" full body -battle -compilation',
        '"tutting" dance tutorial solo -battle', '"top rock" breakdance tutorial -battle -redbull',
        '"liquid dance" tutorial solo -battle', '"shuffling footwork" tutorial -festival -compilation',
        '"krumping" tutorial solo -battle', '"contemporary dance" improv solo -stage'
    ],
    "precision_tools": [
        '"opening door" pantomime -movie -vlog', '"picking up cup" pantomime -review',
        '"hammer swing" technique solo -review -construction', '"steering wheel" turning technique -driving_test -vlog',
        '"pouring liquid" technique hand -review', '"typing on keyboard" close up hand -review',
        '"sweeping floor" full body -vlog'
    ],
    "precision_gestures": [
        '"waving hand gesture" -animation -green_screen', '"pointing gesture" tutorial -body_language_analysis',
        '"thumbs up" gesture tutorial solo', '"clapping hands" technique slow motion',
        '"salute" gesture military -movie -ceremony', '"finger isolation" exercises hand',
        '"handshake" gesture tutorial solo', '"peace sign" gesture tutorial solo'
    ]
}

# ----------------------------------------------------------------------
# 3. FILTERING ENGINE (Duration, Live Stream, 80% Deduplication)
# ----------------------------------------------------------------------
def create_match_filter(existing_titles_list):
    def match_filter(info_dict, *args, **kwargs):
        if info_dict.get('is_live'):
            return "Skipping (Live Stream)"

        duration = info_dict.get('duration')
        if duration and not (10 <= duration <= 900):
            return f"Skipping (Duration: {duration}s)"

        incoming_title = info_dict.get('title', '')
        if not incoming_title:
            return None

        incoming_lower = incoming_title.lower()
        for archived_title in existing_titles_list:
            similarity = difflib.SequenceMatcher(None, incoming_lower, archived_title.lower()).ratio()
            if similarity >= 0.80:
                return f"Skipping (80%+ similar to: '{archived_title[:40]}...')"

        return None
    return match_filter

# ----------------------------------------------------------------------
# 4. EXECUTION ENGINE
# ----------------------------------------------------------------------
def process_query(query: str, target_downloads: int, download: bool):
    # Over-Search: Fetch 3x the target to allow filters room to discard garbage
    search_depth = target_downloads * 3 
    search_url = f"ytsearch{search_depth}:{query}"
    
    ydl_opts = {
        'format': 'bestvideo[height<=1080][fps>=60]/bestvideo[height<=1080][fps>=30]/bestvideo[height<=1080]',
        'outtmpl': os.path.join(DOWNLOAD_DIR, '%(id)s_%(title).50s.%(ext)s'),
        'ffmpeg_location': REQ_DIR,  
        'noplaylist': False, 
        'quiet': True,
        'no_warnings': True,
        'ignoreerrors': True,
        'match_filter': create_match_filter(EXISTING_TITLES),
        'download_archive': os.path.join(DOWNLOAD_DIR, 'yt_archive.txt'),
        
        # 🚨 PIPELINE FIX 1: Strip emojis and weird characters for OpenCV compatibility
        'restrictfilenames': True, 
        
        # 🚨 PIPELINE FIX 2: Network resilience for overnight runs
        'socket_timeout': 15,
        'retries': 10,

        'sleep_interval_requests': 1.0,
        'sleep_interval': 2.0,
        'max_sleep_interval': 5.0,
        'max_downloads': target_downloads, 
    }

    def progress_hook(d):
        if d['status'] == 'finished':
            title_str = d['info_dict'].get('title', 'Unknown')
            print(f"     ✅ Saved: {title_str[:60]}...")
            
            # 🚨 PIPELINE FIX 3: Thread-Safe Atomic Writing
            with ARCHIVE_LOCK: 
                if title_str not in EXISTING_TITLES:
                    EXISTING_TITLES.append(title_str)
                    temp_archive = TITLE_ARCHIVE_FILE + ".tmp"
                    with open(temp_archive, 'w', encoding='utf-8') as f:
                        json.dump(EXISTING_TITLES, f, indent=4)
                    os.replace(temp_archive, TITLE_ARCHIVE_FILE)

    ydl_opts['progress_hooks'] = [progress_hook]

    if not download:
        ydl_opts['skip_download'] = True

    print(f"  🔍 Sniping: '{query}'")
    
    with yt_dlp.YoutubeDL(ydl_opts) as ydl:
        try:
            ydl.download([search_url])
        except Exception as e:
            print(f"  ❌ Error processing query: {e}")

def main():
    parser = argparse.ArgumentParser(description="Portable yt-dlp Harvester for RevExBot.")
    parser.add_argument("--download", action="store_true", help="Actually download videos.")
    parser.add_argument("--max-results", type=int, default=12, help="Target downloads per query (Default 12).")
    args = parser.parse_args()

    all_queries = [q for sublist in SEARCH_QUERIES.values() for q in sublist]
    target_total = len(all_queries) * args.max_results

    print(f"🚀 Initializing Portable Harvester...")
    print(f"⚙️  FFmpeg Path: {REQ_DIR}")
    print(f"📂 Loaded {len(EXISTING_TITLES)} titles from deduplication archive.")
    print(f"🎯 86 Queries -> Targeting {args.max_results} valid downloads each (~{target_total} total).\n")

    for idx, query in enumerate(all_queries, 1):
        print(f"[{idx}/{len(all_queries)}]", end="")
        process_query(query, args.max_results, args.download)
        # 🚨 PIPELINE FIX 4: Randomized human mimicry (between 1.5 and 3.5 seconds)
        time.sleep(random.uniform(1.5, 3.5))
        
    print(f"\n🏁 Harvester Complete. Total unique titles archived: {len(EXISTING_TITLES)}")

if __name__ == "__main__":
    main()
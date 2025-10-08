// chess_assistant_single.cpp
// Build: g++ -std=gnu++17 -O2 chess_assistant_single.cpp -o chess_assistant
// Windows (MSVC): cl /std:c++17 chess_assistant_single.cpp
// Usage:
//   ./chess_assistant --sf ./stockfish [--lc0 ./lc0] [--fen "<FEN>"] [--movetime 2000] [--multipv 3]
//   ./chess_assistant --sf ./stockfish --startpos --moves "e2e4 e7e5 g1f3" --movetime 3000 --multipv 4
//
// Notes:
// - If you supply --lc0, it will run both engines in parallel and fuse outputs.
// - For endgames, enable Syzygy in your engines via their own options (or set through setOption).
// - This is analysis-only; do not use it for real-time assistance in rated games.

#include <bits/stdc++.h>
#ifdef _WIN32
#define NOMINMAX
#include <windows.h>
#else
#include <unistd.h>
#include <sys/types.h>
#include <sys/wait.h>
#include <fcntl.h>
#endif

// --------------------------- Utility types ---------------------------

struct PvLine {
    int depth = 0;
    int multiPvIndex = 1;
    int scoreCp = 0;        // centipawns (if not mate)
    int scoreMate = 0;      // mate in N; positive for mate in N, negative for mate for opponent
    double nps = 0.0;       // nodes per second
    std::string pv;         // principal variation (space-separated moves)
};

struct EngineEval {
    std::vector<PvLine> lines;  // collected info snapshots (you can take last per MultiPV)
    std::string bestMove;
    bool ready = false;
};

// --------------------------- Process wrapper ---------------------------

class ChildProcess {
public:
    explicit ChildProcess(const std::string& path) : path_(path) {}
    bool start() {
#ifdef _WIN32
        SECURITY_ATTRIBUTES sa; ZeroMemory(&sa, sizeof(sa));
        sa.nLength = sizeof(SECURITY_ATTRIBUTES);
        sa.bInheritHandle = TRUE;

        HANDLE outRead = NULL, outWrite = NULL;
        HANDLE inRead = NULL, inWrite = NULL;

        if (!CreatePipe(&outRead, &outWrite, &sa, 0)) return false;
        if (!SetHandleInformation(outRead, HANDLE_FLAG_INHERIT, 0)) return false;
        if (!CreatePipe(&inRead, &inWrite, &sa, 0)) return false;
        if (!SetHandleInformation(inWrite, HANDLE_FLAG_INHERIT, 0)) return false;

        STARTUPINFOA si; ZeroMemory(&si, sizeof(si));
        si.cb = sizeof(si);
        si.hStdInput = inRead;
        si.hStdOutput = outWrite;
        si.hStdError = outWrite;
        si.dwFlags = STARTF_USESTDHANDLES;

        PROCESS_INFORMATION pi; ZeroMemory(&pi, sizeof(pi));

        BOOL ok = CreateProcessA(path_.c_str(), NULL, NULL, NULL, TRUE, 0, NULL, NULL, &si, &pi);
        if (!ok) {
            CloseHandle(outRead); CloseHandle(outWrite);
            CloseHandle(inRead);  CloseHandle(inWrite);
            return false;
        }

        CloseHandle(outWrite);
        CloseHandle(inRead);

        childStdout_ = outRead;
        childStdin_  = inWrite;
        proc_ = pi;
        return true;
#else
        int inPipe[2], outPipe[2];
        if (pipe(inPipe) < 0 || pipe(outPipe) < 0) return false;

        pid_ = fork();
        if (pid_ == 0) {
            dup2(outPipe[0], STDIN_FILENO);
            dup2(inPipe[1], STDOUT_FILENO);
            dup2(inPipe[1], STDERR_FILENO);
            close(outPipe[1]); close(inPipe[0]);
            execl(path_.c_str(), path_.c_str(), (char*)NULL);
            std::exit(1);
        }
        close(outPipe[0]); close(inPipe[1]);
        stdinFd_  = outPipe[1];
        stdoutFd_ = inPipe[0];
        fcntl(stdoutFd_, F_SETFL, O_NONBLOCK);
        return true;
#endif
    }

    void writeLine(const std::string& s) {
        std::string msg = s + "\n";
#ifdef _WIN32
        DWORD written = 0;
        WriteFile(childStdin_, msg.c_str(), (DWORD)msg.size(), &written, NULL);
#else
        ::write(stdinFd_, msg.c_str(), msg.size());
#endif
    }

    // Read any available chunk; caller should parse line boundaries.
    std::string readChunk() {
#ifdef _WIN32
        char buf[8192];
        DWORD read = 0;
        if (!ReadFile(childStdout_, buf, sizeof(buf) - 1, &read, NULL) || read == 0) return "";
        buf[read] = '\0';
        return std::string(buf);
#else
        char buf[8192];
        int n = ::read(stdoutFd_, buf, sizeof(buf) - 1);
        if (n <= 0) return "";
        buf[n] = '\0';
        return std::string(buf);
#endif
    }

    void stop() {
#ifdef _WIN32
        if (childStdin_) {
            writeLine("quit");
            CloseHandle(childStdin_);
            childStdin_ = NULL;
        }
        if (childStdout_) { CloseHandle(childStdout_); childStdout_ = NULL; }
        if (proc_.hProcess) {
            WaitForSingleObject(proc_.hProcess, 100);
            TerminateProcess(proc_.hProcess, 0);
            CloseHandle(proc_.hProcess);
            CloseHandle(proc_.hThread);
            ZeroMemory(&proc_, sizeof(proc_));
        }
#else
        if (stdinFd_ >= 0) {
            writeLine("quit");
            close(stdinFd_); stdinFd_ = -1;
        }
        if (stdoutFd_ >= 0) { close(stdoutFd_); stdoutFd_ = -1; }
        if (pid_ > 0) { waitpid(pid_, nullptr, 0); pid_ = -1; }
#endif
    }

private:
    std::string path_;
#ifdef _WIN32
    PROCESS_INFORMATION proc_{};
    HANDLE childStdout_{NULL};
    HANDLE childStdin_{NULL};
#else
    pid_t pid_ = -1;
    int stdinFd_ = -1;
    int stdoutFd_ = -1;
#endif
};

// --------------------------- UCI engine ---------------------------

class UciEngine {
public:
    explicit UciEngine(const std::string& path) : path_(path), proc_(path) {}

    bool start() {
        if (!proc_.start()) return false;
        write("uci");
        if (!waitFor("uciok", 2000)) return false;
        write("isready");
        return waitFor("readyok", 2000);
    }

    void setOption(const std::string& name, const std::string& value) {
        write("setoption name " + name + " value " + value);
    }

    void newGame() {
        write("ucinewgame");
        write("isready");
        waitFor("readyok", 2000);
    }

    void positionStartpos(const std::vector<std::string>& moves) {
        std::string cmd = "position startpos";
        if (!moves.empty()) {
            cmd += " moves";
            for (auto& m : moves) cmd += " " + m;
        }
        write(cmd);
    }

    void positionFEN(const std::string& fen, const std::vector<std::string>& moves) {
        std::string cmd = "position fen " + fen;
        if (!moves.empty()) {
            cmd += " moves";
            for (auto& m : moves) cmd += " " + m;
        }
        write(cmd);
    }

    EngineEval goMovetime(int ms, int multipv) {
        write("setoption name MultiPV value " + std::to_string(multipv));
        write("go movetime " + std::to_string(ms));
        EngineEval eval;
        auto deadline = std::chrono::steady_clock::now() + std::chrono::milliseconds(ms + 1500);
        std::string buffer;
        while (std::chrono::steady_clock::now() < deadline) {
            std::string chunk = proc_.readChunk();
            if (chunk.empty()) {
                std::this_thread::sleep_for(std::chrono::milliseconds(5));
                continue;
            }
            buffer += chunk;
            // Split lines
            size_t pos = 0;
            while (true) {
                size_t nl = buffer.find('\n', pos);
                if (nl == std::string::npos) {
                    // keep partial
                    buffer = buffer.substr(pos);
                    break;
                }
                std::string line = buffer.substr(pos, nl - pos);
                pos = nl + 1;
                parseLine(line, eval);
                if (line.rfind("bestmove", 0) == 0) {
                    eval.ready = true;
                    return eval;
                }
            }
        }
        return eval;
    }

    void stop() { proc_.stop(); }

private:
    std::string path_;
    ChildProcess proc_;

    void write(const std::string& s) { proc_.writeLine(s); }

    bool waitFor(const std::string& token, int timeoutMs) {
        auto deadline = std::chrono::steady_clock::now() + std::chrono::milliseconds(timeoutMs);
        std::string buf;
        while (std::chrono::steady_clock::now() < deadline) {
            std::string chunk = proc_.readChunk();
            if (!chunk.empty()) {
                buf += chunk;
                if (buf.find(token) != std::string::npos) return true;
            } else {
                std::this_thread::sleep_for(std::chrono::milliseconds(5));
            }
        }
        return false;
    }

    static int parseInt(const std::string& s) {
        try { return std::stoi(s); } catch (...) { return 0; }
    }

    void parseLine(const std::string& line, EngineEval& out) {
        if (line.rfind("info", 0) == 0) {
            PvLine pv;
            // tokenized parse
            std::istringstream iss(line);
            std::string tok;
            while (iss >> tok) {
                if (tok == "depth") { iss >> pv.depth; }
                else if (tok == "multipv") { iss >> pv.multiPvIndex; }
                else if (tok == "score") {
                    std::string kind; iss >> kind;
                    if (kind == "cp") { iss >> pv.scoreCp; }
                    else if (kind == "mate") { iss >> pv.scoreMate; }
                } else if (tok == "nps") {
                    std::string npsStr; iss >> npsStr;
                    pv.nps = atof(npsStr.c_str());
                } else if (tok == "pv") {
                    // rest of line is PV
                    std::string rest; std::getline(iss, rest);
                    // trim leading space
                    if (!rest.empty() && rest[0] == ' ') rest.erase(0, 1);
                    pv.pv = rest;
                    break;
                }
            }
            if (!pv.pv.empty() || pv.depth > 0) out.lines.push_back(pv);
        } else if (line.rfind("bestmove", 0) == 0) {
            // format: bestmove <uci> [ponder <uci>]
            std::istringstream iss(line);
            std::string bm; iss >> bm; // "bestmove"
            std::string move; iss >> move;
            out.bestMove = move;
        }
    }
};

// --------------------------- Meta-policy broker ---------------------------

class EngineBroker {
public:
    EngineBroker(const std::string& sfPath, const std::string& lc0Path)
        : sf_(sfPath), lc0Path_(lc0Path), useLc0_(!lc0Path.empty()), lc0_(lc0Path.empty() ? "" : lc0Path) {}

    bool start() {
        bool ok1 = sf_.start();
        if (!ok1) return false;
        // Sensible defaults; tune per hardware
        sf_.setOption("Threads", std::to_string(std::thread::hardware_concurrency() ? std::thread::hardware_concurrency() : 4));
        sf_.setOption("Hash", "1024");
        if (useLc0_) {
            bool ok2 = lc0_.start();
            if (!ok2) { useLc0_ = false; }
        }
        return true;
    }

    void newGame() { sf_.newGame(); if (useLc0_) lc0_.newGame(); }

    std::pair<EngineEval, EngineEval> analyzeFEN(const std::string& fen, int ms, int multipv,
                                                 const std::vector<std::string>& moves) {
        sf_.positionFEN(fen, moves);
        EngineEval sfEval, lcEval;
        std::atomic<bool> sfDone{false}, lcDone{!useLc0_};

        std::thread tSF([&](){ sfEval = sf_.goMovetime(ms, multipv); sfDone = true; });
        std::thread tLc;
        if (useLc0_) {
            lc0_.positionFEN(fen, moves);
            tLc = std::thread([&](){ lcEval = lc0_.goMovetime(ms, multipv); lcDone = true; });
        }

        while (!(sfDone.load() && lcDone.load())) { std::this_thread::sleep_for(std::chrono::milliseconds(10)); }
        tSF.join(); if (useLc0_) tLc.join();
        return {sfEval, lcEval};
    }

    std::string selectMove(const EngineEval& sfEval, const EngineEval& lcEval) {
        if (!useLc0_) return sfEval.bestMove;
        if (sfEval.bestMove == lcEval.bestMove && !sfEval.bestMove.empty()) return sfEval.bestMove;

        int sfScore = topScore(sfEval.lines);
        int lcScore = topScore(lcEval.lines);

        // Heuristic: prefer higher mate score, then higher cp; tie-break by depth
        auto [sfMate, sfDepth] = topMateDepth(sfEval.lines);
        auto [lcMate, lcDepth] = topMateDepth(lcEval.lines);

        if (sfMate != 0 || lcMate != 0) {
            if (sfMate != 0 && lcMate != 0) return (abs(sfMate) <= abs(lcMate) ? sfEval.bestMove : lcEval.bestMove);
            return sfMate != 0 ? sfEval.bestMove : lcEval.bestMove;
        }
        if (sfScore != lcScore) return (sfScore >= lcScore ? sfEval.bestMove : lcEval.bestMove);
        return (sfDepth >= lcDepth ? sfEval.bestMove : lcEval.bestMove);
    }

    void stop() { sf_.stop(); if (useLc0_) lc0_.stop(); }

private:
    UciEngine sf_;
    std::string lc0Path_;
    bool useLc0_;
    UciEngine lc0_;

    static int topScore(const std::vector<PvLine>& lines) {
        int best = -10000000;
        for (auto& l : lines) {
            int s = (l.scoreMate != 0 ? (200000 - std::abs(l.scoreMate) * 1000) * (l.scoreMate > 0 ? 1 : -1) : l.scoreCp);
            best = std::max(best, s);
        }
        return best;
    }

    static std::pair<int,int> topMateDepth(const std::vector<PvLine>& lines) {
        int mate = 0, depth = 0;
        for (auto& l : lines) {
            if (l.scoreMate != 0) {
                if (mate == 0 || std::abs(l.scoreMate) < std::abs(mate)) { mate = l.scoreMate; depth = l.depth; }
            }
        }
        return {mate, depth};
    }
};

// --------------------------- Explainer ---------------------------

enum Motif { TacticFork, TacticPin, BackRank, PassedPawnPush, KingAttack, ExchangeSac, Prophylaxis, EndgameTechnique, SpaceGain };

static std::string motifName(Motif m) {
    switch (m) {
        case TacticFork: return "Fork";
        case TacticPin: return "Pin";
        case BackRank: return "Back‑rank";
        case PassedPawnPush: return "Passed pawn push";
        case KingAttack: return "King attack";
        case ExchangeSac: return "Exchange sacrifice";
        case Prophylaxis: return "Prophylaxis";
        case EndgameTechnique: return "Endgame technique";
        case SpaceGain: return "Space gain";
        default: return "Motif";
    }
}

struct Explanation {
    std::string summary;
    std::vector<Motif> motifs;
    std::vector<std::string> turningPoints; // "move: eval delta"
};

static std::vector<std::string> splitPV(const std::string& pv) {
    std::vector<std::string> mv;
    std::istringstream iss(pv);
    std::string m;
    while (iss >> m) mv.push_back(m);
    return mv;
}

Explanation explainFromLines(const std::vector<PvLine>& lines) {
    Explanation e;
    if (lines.empty()) { e.summary = "No analysis info available."; return e; }

    // Take top PV by highest depth and then best score
    const PvLine* top = nullptr;
    for (auto& l : lines) {
        if (!top || l.depth > top->depth || (l.depth == top->depth && (l.scoreMate != 0 || l.scoreCp > top->scoreCp))) top = &l;
    }
    auto moves = splitPV(top ? top->pv : "");
    // Heuristics: look for pawn pushes, checks, captures, quiet moves
    int pawnPushes = 0, checks = 0, captures = 0;
    for (auto& m : moves) {
        if (m.size() >= 4 && (m[1] == '2' || m[1] == '3' || m[1] == '4' || m[1] == '5')) {
            // not reliable without board; heuristic
        }
        if (m.find('+') != std::string::npos || m.find('#') != std::string::npos) checks++;
        if (m.find('x') != std::string::npos) captures++;
        // pawn push heuristic: move starts with file letter and goes two ranks (e2e4)
        if (m.size() >= 4 && isalpha(m[0]) && isdigit(m[1]) && isalpha(m[2]) && isdigit(m[3])) {
            if (m[0] == m[2] && (m[3] - m[1]) != 0) pawnPushes++;
        }
    }

    // Motif tagging
    if (checks >= 2) e.motifs.push_back(KingAttack);
    if (captures >= 2 && checks == 0) e.motifs.push_back(EndgameTechnique);
    if (pawnPushes >= 2) e.motifs.push_back(PassedPawnPush);
    if (top && top->depth >= 20 && std::abs(top->scoreCp) < 40 && checks == 0) e.motifs.push_back(Prophylaxis);
    if (top && std::abs(top->scoreCp) > 120) e.motifs.push_back(SpaceGain);

    // Summary
    std::ostringstream ss;
    ss << "Plan: ";
    bool first = true;
    auto add = [&](const std::string& s){ if (!first) ss << ", "; ss << s; first = false; };
    for (auto m : e.motifs) add(motifName(m));
    if (first) add("Improve pieces and restrict counterplay"); // fallback
    e.summary = ss.str();

    // Turning points: scan lines for big eval swings (requires chronological info; approximate by top snapshots)
    for (size_t i = 1; i < lines.size(); ++i) {
        int prev = lines[i-1].scoreMate ? lines[i-1].scoreMate*1000 : lines[i-1].scoreCp;
        int cur  = lines[i].scoreMate ? lines[i].scoreMate*1000 : lines[i].scoreCp;
        int delta = cur - prev;
        if (std::abs(delta) >= 150) {
            std::ostringstream tp;
            tp << "Depth " << lines[i].depth << ": eval shift " << (delta >= 0 ? "+" : "") << delta/100.0 << " cp";
            e.turningPoints.push_back(tp.str());
        }
    }
    return e;
}

// --------------------------- CLI & main ---------------------------

struct Args {
    std::string sfPath;
    std::string lc0Path;
    std::string fen;
    std::vector<std::string> moves;
    bool useStartpos = false;
    int movetimeMs = 2000;
    int multipv = 3;
};

Args parseArgs(int argc, char** argv) {
    Args a;
    for (int i = 1; i < argc; ++i) {
        std::string s = argv[i];
        if (s == "--sf" && i+1 < argc) a.sfPath = argv[++i];
        else if (s == "--lc0" && i+1 < argc) a.lc0Path = argv[++i];
        else if (s == "--fen" && i+1 < argc) a.fen = argv[++i];
        else if (s == "--startpos") a.useStartpos = true;
        else if (s == "--moves" && i+1 < argc) {
            // space-separated UCI moves
            std::istringstream iss(argv[++i]); std::string m;
            while (iss >> m) a.moves.push_back(m);
        }
        else if (s == "--movetime" && i+1 < argc) a.movetimeMs = std::max(100, atoi(argv[++i]));
        else if (s == "--multipv" && i+1 < argc) a.multipv = std::max(1, atoi(argv[++i]));
    }
    return a;
}

void printEval(const char* tag, const EngineEval& eval) {
    std::cout << "=== " << tag << " ===\n";
    if (!eval.bestMove.empty()) std::cout << "Best move: " << eval.bestMove << "\n";
    // Aggregate last PV per MultiPV index
    std::map<int, PvLine> lastPV;
    for (auto& l : eval.lines) lastPV[l.multiPvIndex] = l;
    for (auto& [idx, l] : lastPV) {
        std::cout << "PV#" << idx
                  << " depth " << l.depth
                  << " score " << (l.scoreMate ? ("mate " + std::to_string(l.scoreMate))
                                               : (std::to_string(l.scoreCp) + " cp"))
                  << " nps " << std::fixed << std::setprecision(0) << l.nps
                  << "\n  " << l.pv << "\n";
    }
    Explanation ex = explainFromLines(eval.lines);
    std::cout << "Summary: " << ex.summary << "\n";
    if (!ex.motifs.empty()) {
        std::cout << "Motifs: ";
        for (size_t i = 0; i < ex.motifs.size(); ++i) {
            if (i) std::cout << ", ";
            std::cout << motifName(ex.motifs[i]);
        }
        std::cout << "\n";
    }
    if (!ex.turningPoints.empty()) {
        std::cout << "Turning points:\n";
        for (auto& t : ex.turningPoints) std::cout << "  - " << t << "\n";
    }
}

int main(int argc, char** argv) {
    Args args = parseArgs(argc, argv);
    if (args.sfPath.empty()) {
        std::cerr << "Usage: " << argv[0] << " --sf <stockfish_path> [--lc0 <lc0_path>] "
                  << "[--fen \"<FEN>\" | --startpos] [--moves \"m1 m2 ...\"] "
                  << "[--movetime 2000] [--multipv 3]\n";
        return 1;
    }

    EngineBroker broker(args.sfPath, args.lc0Path);
    if (!broker.start()) {
        std::cerr << "Failed to start engines.\n";
        return 1;
    }
    broker.newGame();

    std::string fen = args.fen;
    if (args.useStartpos) fen = "startpos";
    bool useStartpos = args.useStartpos || args.fen.empty();

    // Positioning
    std::pair<EngineEval, EngineEval> result;
    if (useStartpos) {
        // For simplicity: translate to FEN via engine's "position startpos"
        // We’ll just analyze with FEN set to default; engines use internal pos state per command.
        // Here we call analyzeFEN with an internal assumption; engines already received position in analyzeFEN.
        // To keep single-file, we pass moves to engines directly there.
    }

    // Run analysis
    if (useStartpos) {
        // Engines set startpos inside analyze; we emulate by using the same FEN command path
        // Easiest approach: build a FEN for startpos; engines accept "position startpos", but our API uses FEN.
        // We’ll send "position FEN <startpos>" by using the standard initial FEN.
        std::string startFEN = "rnbqkbnr/pppppppp/8/8/8/8/PPPPPPPP/RNBQKBNR w KQkq - 0 1";
        result = broker.analyzeFEN(startFEN, args.movetimeMs, args.multipv, args.moves);
    } else {
        result = broker.analyzeFEN(args.fen, args.movetimeMs, args.multipv, args.moves);
    }

    auto& sfEval = result.first;
    auto& lcEval = result.second;

    printEval("Stockfish", sfEval);
    if (!args.lc0Path.empty() && !lcEval.lines.empty()) printEval("Lc0", lcEval);

    std::string chosen = broker.selectMove(sfEval, lcEval);
    std::cout << ">>> Selected move (meta‑policy): " << (chosen.empty() ? sfEval.bestMove : chosen) << "\n";

    broker.stop();
    return 0;
}

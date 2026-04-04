"use strict";

(function () {
  const INDEX_PATH = "_static/section-search-index.json";
  const INDEX_SCRIPT_PATH = "_static/section-search-index.js";
  const SEARCH_APP_ID = "section-search-app";
  const SEARCH_RESULTS_ID = "section-search-results";
  const SEARCH_STATUS_ID = "section-search-status";
  const SEARCH_HISTORY_ID = "section-search-history";
  const HIT_CLASS = "section-search-hit";
  const HISTORY_LIMIT = 8;

  let searchIndexPromise = null;

  document.addEventListener("DOMContentLoaded", () => {
    initSearchPage();
    initContentHighlight();
  });

  function initSearchPage() {
    const app = document.getElementById(SEARCH_APP_ID);
    if (!app) {
      return;
    }

    const query = getQueryParam("q").trim();
    const form = app.querySelector("form");
    const input = form ? form.querySelector('input[name="q"]') : null;
    const resultsRoot = document.getElementById(SEARCH_RESULTS_ID);
    const statusRoot = document.getElementById(SEARCH_STATUS_ID);
    const historyRoot = document.getElementById(SEARCH_HISTORY_ID);

    if (input) {
      input.value = query;
    }

    initSearchHistory(app, form, input, historyRoot, query);

    if (!query) {
      if (statusRoot) {
        statusRoot.textContent = app.dataset.emptyQuery || "";
      }
      return;
    }

    if (statusRoot) {
      statusRoot.textContent = app.dataset.loading || "";
    }

    loadSearchIndex()
      .then((payload) => {
        const results = searchEntries(payload.entries || [], query);
        saveSearchHistory(query);
        renderSearchHistory(app, historyRoot, query, input ? input.value.trim() : "");
        renderResults(resultsRoot, statusRoot, app, query, results);
      })
      .catch((error) => {
        console.error("Failed to load section search index.", error);
        if (statusRoot) {
          statusRoot.textContent = app.dataset.loadError || "Search index failed to load.";
        }
      });
  }

  function initContentHighlight() {
    const app = document.getElementById(SEARCH_APP_ID);
    if (app) {
      return;
    }

    const query = getQueryParam("highlight").trim();
    if (!query) {
      return;
    }

    const container = document.querySelector('[itemprop="articleBody"]');
    if (!container) {
      return;
    }

    const terms = buildHighlightTerms(query);
    if (!terms.length) {
      return;
    }

    highlightMatches(container, terms);
    scrollToClosestHit();
  }

  function loadSearchIndex() {
    if (window.SECTION_SEARCH_INDEX) {
      return Promise.resolve(window.SECTION_SEARCH_INDEX);
    }

    if (!searchIndexPromise) {
      searchIndexPromise = loadSearchIndexScript().catch(() => loadSearchIndexJson());
    }
    return searchIndexPromise;
  }

  function loadSearchIndexScript() {
    return new Promise((resolve, reject) => {
      if (window.SECTION_SEARCH_INDEX) {
        resolve(window.SECTION_SEARCH_INDEX);
        return;
      }

      const existing = document.querySelector('script[data-section-search-index="true"]');
      if (existing) {
        existing.addEventListener("load", () => resolve(window.SECTION_SEARCH_INDEX));
        existing.addEventListener("error", reject);
        return;
      }

      const script = document.createElement("script");
      script.src = getUrlRoot() + INDEX_SCRIPT_PATH;
      script.async = true;
      script.dataset.sectionSearchIndex = "true";
      script.onload = () => {
        if (window.SECTION_SEARCH_INDEX) {
          resolve(window.SECTION_SEARCH_INDEX);
        } else {
          reject(new Error("Section search index script loaded without payload."));
        }
      };
      script.onerror = () => reject(new Error("Failed to load section search index script."));
      document.head.appendChild(script);
    });
  }

  function loadSearchIndexJson() {
    return fetch(getUrlRoot() + INDEX_PATH).then((response) => {
      if (!response.ok) {
        throw new Error("HTTP " + response.status);
      }
      return response.json();
    });
  }

  function searchEntries(entries, rawQuery) {
    const query = normalizeText(rawQuery);
    const tokens = buildSearchTokens(rawQuery);

    return entries
      .map((entry) => scoreEntry(entry, query, tokens))
      .filter(Boolean)
      .sort(compareResults)
      .slice(0, 80);
  }

  function scoreEntry(entry, query, tokens) {
    const title = normalizeText((entry.title || "") + " " + (entry.page_title || ""));
    const text = normalizeText(entry.text || "");
    const combined = (title + "\n" + text).trim();

    if (!combined) {
      return null;
    }

    const phraseMatch = query && combined.includes(query);
    const matchedTokens = tokens.filter((token) => title.includes(token) || text.includes(token));

    if (!phraseMatch && tokens.length && matchedTokens.length < tokens.length) {
      return null;
    }

    if (!phraseMatch && !matchedTokens.length) {
      return null;
    }

    let score = entry.kind === "section" ? 20 : 5;

    if (query && title.includes(query)) {
      score += 160;
    }
    if (query && text.includes(query)) {
      score += 90;
    }

    matchedTokens.forEach((token) => {
      if (title.includes(token)) {
        score += 45;
      } else {
        score += 18;
      }
    });

    if (entry.page_title && entry.title && entry.page_title !== entry.title) {
      score += 10;
    }

    return {
      entry,
      score,
      snippet: makeSnippet(entry.text || entry.page_title || entry.title || "", query, tokens),
    };
  }

  function compareResults(left, right) {
    if (right.score !== left.score) {
      return right.score - left.score;
    }

    const leftTitle = ((left.entry.title || "") + " " + (left.entry.page_title || "")).toLowerCase();
    const rightTitle = ((right.entry.title || "") + " " + (right.entry.page_title || "")).toLowerCase();
    return leftTitle.localeCompare(rightTitle);
  }

  function renderResults(root, statusRoot, app, query, results) {
    if (!root) {
      return;
    }

    const highlightTerms = buildHighlightTerms(query);
    root.replaceChildren();

    if (!results.length) {
      if (statusRoot) {
        statusRoot.textContent = app.dataset.noResults || "";
      }
      return;
    }

    if (statusRoot) {
      statusRoot.textContent = formatCountMessage(app, results.length);
    }

    const list = document.createElement("ul");
    list.className = "search";
    list.setAttribute("role", "list");

    results.forEach((result) => {
      list.appendChild(renderResultItem(result, query, highlightTerms));
    });

    root.appendChild(list);
  }

  function initSearchHistory(app, form, input, historyRoot, query) {
    if (!form || !input || !historyRoot) {
      return;
    }

    renderSearchHistory(app, historyRoot, query, input.value.trim());

    form.addEventListener("submit", () => {
      saveSearchHistory(input.value);
    });

    input.addEventListener("focus", () => {
      renderSearchHistory(app, historyRoot, query, input.value.trim());
      toggleHistoryVisibility(historyRoot, true);
    });

    input.addEventListener("input", () => {
      renderSearchHistory(app, historyRoot, query, input.value.trim());
      toggleHistoryVisibility(historyRoot, true);
    });

    document.addEventListener("click", (event) => {
      if (!historyRoot.contains(event.target) && !form.contains(event.target)) {
        toggleHistoryVisibility(historyRoot, false);
      }
    });
  }

  function renderSearchHistory(app, historyRoot, currentQuery, draftQuery) {
    if (!historyRoot) {
      return;
    }

    const history = getSearchHistory();
    const keyword = normalizeText(draftQuery || "");
    const filtered = keyword
      ? history.filter((item) => normalizeText(item).includes(keyword))
      : history.slice();

    historyRoot.replaceChildren();

    const header = document.createElement("div");
    header.className = "section-search-history__header";

    const title = document.createElement("div");
    title.className = "section-search-history__title";
    title.textContent = app.dataset.historyTitle || "Recent searches";
    header.appendChild(title);

    if (history.length) {
      const clearButton = document.createElement("button");
      clearButton.type = "button";
      clearButton.className = "section-search-history__clear";
      clearButton.textContent = app.dataset.clearHistory || "Clear history";
      clearButton.addEventListener("click", () => {
        clearSearchHistory();
        renderSearchHistory(app, historyRoot, currentQuery, draftQuery);
        toggleHistoryVisibility(historyRoot, true);
      });
      header.appendChild(clearButton);
    }

    historyRoot.appendChild(header);

    const list = document.createElement("div");
    list.className = "section-search-history__list";

    if (!filtered.length) {
      const empty = document.createElement("div");
      empty.className = "section-search-history__empty";
      empty.textContent = keyword ? app.dataset.noResults || "" : app.dataset.historyEmpty || "";
      list.appendChild(empty);
    } else {
      filtered.forEach((item) => {
        const link = document.createElement("a");
        link.className = "section-search-history__item";
        link.href = "?q=" + encodeURIComponent(item);
        link.textContent = item;
        if (normalizeText(item) === normalizeText(currentQuery)) {
          link.setAttribute("aria-current", "true");
        }
        list.appendChild(link);
      });
    }

    historyRoot.appendChild(list);
    toggleHistoryVisibility(historyRoot, Boolean(history.length || keyword));
  }

  function renderResultItem(result, query, highlightTerms) {
    const item = document.createElement("li");
    item.className = "kind-text section-search-result";

    const header = document.createElement("div");
    header.className = "section-search-result__header";
    const link = document.createElement("a");
    link.className = "section-search-result__title";
    link.href = buildResultUrl(result.entry.url, query);
    appendHighlightedText(link, result.entry.title || result.entry.page_title || "", highlightTerms);
    header.appendChild(link);
    item.appendChild(header);

    if (result.entry.page_title && result.entry.page_title !== result.entry.title) {
      const context = document.createElement("div");
      context.className = "section-search-result__meta";
      context.appendChild(createHighlightedFragment(result.entry.page_title, highlightTerms));
      item.appendChild(context);
    }

    if (result.snippet) {
      const summary = document.createElement("p");
      summary.className = "search-snippet section-search-result__snippet";
      summary.appendChild(createHighlightedFragment(result.snippet, highlightTerms));
      item.appendChild(summary);
    }

    return item;
  }

  function buildResultUrl(url, query) {
    const hashIndex = url.indexOf("#");
    const base = hashIndex >= 0 ? url.slice(0, hashIndex) : url;
    const hash = hashIndex >= 0 ? url.slice(hashIndex) : "";
    const separator = base.includes("?") ? "&" : "?";
    return base + separator + "highlight=" + encodeURIComponent(query) + hash;
  }

  function makeSnippet(text, query, tokens) {
    const source = collapseWhitespace(text);
    if (!source) {
      return "";
    }

    const candidates = [query].concat(tokens).filter(Boolean);
    let index = -1;

    for (const term of candidates) {
      index = source.toLowerCase().indexOf(term.toLowerCase());
      if (index >= 0) {
        break;
      }
    }

    if (index < 0) {
      return source.slice(0, 180) + (source.length > 180 ? "..." : "");
    }

    const start = Math.max(index - 70, 0);
    const end = Math.min(index + 110, source.length);
    const prefix = start > 0 ? "..." : "";
    const suffix = end < source.length ? "..." : "";
    return prefix + source.slice(start, end) + suffix;
  }

  function buildSearchTokens(query) {
    const normalized = normalizeText(query);
    const latinTokens = normalized
      .split(/[^\p{Letter}\p{Number}_]+/u)
      .map((token) => token.trim())
      .filter(Boolean);
    const cjkTokens = normalized.match(/[\u3400-\u9fff]+/gu) || [];

    return Array.from(new Set(latinTokens.concat(cjkTokens)));
  }

  function buildHighlightTerms(query) {
    const normalized = normalizeText(query);
    const tokens = buildSearchTokens(query);
    const ordered = [normalized]
      .concat(tokens)
      .filter(Boolean)
      .sort((left, right) => right.length - left.length);

    return Array.from(new Set(ordered));
  }

  function highlightMatches(container, terms) {
    const matcher = new RegExp("(" + terms.map(escapeRegExp).join("|") + ")", "giu");
    const walker = document.createTreeWalker(container, NodeFilter.SHOW_TEXT, {
      acceptNode(node) {
        const parent = node.parentElement;
        if (!parent || !node.nodeValue || !node.nodeValue.trim()) {
          return NodeFilter.FILTER_REJECT;
        }
        if (parent.closest("script, style, noscript, mark, .headerlink")) {
          return NodeFilter.FILTER_REJECT;
        }
        return containsAny(node.nodeValue, terms) ? NodeFilter.FILTER_ACCEPT : NodeFilter.FILTER_REJECT;
      },
    });

    const textNodes = [];
    while (walker.nextNode()) {
      textNodes.push(walker.currentNode);
    }

    textNodes.forEach((node) => {
      const value = node.nodeValue;
      if (!value) {
        return;
      }

      matcher.lastIndex = 0;
      let lastIndex = 0;
      let match = null;
      const fragment = document.createDocumentFragment();

      while ((match = matcher.exec(value)) !== null) {
        if (match.index > lastIndex) {
          fragment.appendChild(document.createTextNode(value.slice(lastIndex, match.index)));
        }

        const mark = document.createElement("mark");
        mark.className = HIT_CLASS;
        mark.textContent = match[0];
        fragment.appendChild(mark);
        lastIndex = match.index + match[0].length;
      }

      if (lastIndex === 0) {
        return;
      }

      if (lastIndex < value.length) {
        fragment.appendChild(document.createTextNode(value.slice(lastIndex)));
      }

      node.parentNode.replaceChild(fragment, node);
    });
  }

  function scrollToClosestHit() {
    const hash = decodeURIComponent(window.location.hash || "").replace(/^#/, "");
    let target = null;

    if (hash) {
      const anchor = document.getElementById(hash);
      if (anchor) {
        const scope = anchor.closest("section") || anchor;
        target = scope.querySelector("." + HIT_CLASS);
      }
    }

    if (!target) {
      target = document.querySelector("." + HIT_CLASS);
    }

    if (target) {
      window.requestAnimationFrame(() => {
        target.scrollIntoView({ block: "center" });
      });
    }
  }

  function formatCountMessage(app, count) {
    const template = count === 1 ? app.dataset.resultOne : app.dataset.resultMany;
    return (template || "").replace("{count}", String(count));
  }

  function getQueryParam(name) {
    return new URLSearchParams(window.location.search).get(name) || "";
  }

  function getSearchHistory() {
    try {
      const raw = window.localStorage.getItem(getHistoryStorageKey());
      const parsed = raw ? JSON.parse(raw) : [];
      return Array.isArray(parsed)
        ? parsed.map((item) => collapseWhitespace(String(item))).filter(Boolean).slice(0, HISTORY_LIMIT)
        : [];
    } catch (error) {
      return [];
    }
  }

  function saveSearchHistory(query) {
    const normalized = collapseWhitespace(query);
    if (!normalized) {
      return;
    }

    const history = getSearchHistory().filter((item) => item !== normalized);
    history.unshift(normalized);

    try {
      window.localStorage.setItem(getHistoryStorageKey(), JSON.stringify(history.slice(0, HISTORY_LIMIT)));
    } catch (error) {
      console.warn("Failed to save search history.", error);
    }
  }

  function clearSearchHistory() {
    try {
      window.localStorage.removeItem(getHistoryStorageKey());
    } catch (error) {
      console.warn("Failed to clear search history.", error);
    }
  }

  function getHistoryStorageKey() {
    const scope =
      window.location.pathname.replace(/search\.html$/i, "").replace(/index\.html$/i, "") ||
      getUrlRoot() ||
      "default";
    return "section_search_recent_queries::" + scope;
  }

  function getUrlRoot() {
    return (window.DOCUMENTATION_OPTIONS && window.DOCUMENTATION_OPTIONS.URL_ROOT) || "";
  }

  function toggleHistoryVisibility(historyRoot, visible) {
    if (!historyRoot) {
      return;
    }

    historyRoot.hidden = !visible;
  }

  function containsAny(text, terms) {
    const normalized = text.toLowerCase();
    return terms.some((term) => normalized.includes(term.toLowerCase()));
  }

  function appendHighlightedText(target, text, terms) {
    target.appendChild(createHighlightedFragment(text, terms));
  }

  function createHighlightedFragment(text, terms) {
    const fragment = document.createDocumentFragment();
    const source = text || "";
    const filteredTerms = (terms || []).filter(Boolean);

    if (!source || !filteredTerms.length) {
      fragment.appendChild(document.createTextNode(source));
      return fragment;
    }

    const matcher = new RegExp("(" + filteredTerms.map(escapeRegExp).join("|") + ")", "giu");
    let lastIndex = 0;
    let match = null;

    while ((match = matcher.exec(source)) !== null) {
      if (match.index > lastIndex) {
        fragment.appendChild(document.createTextNode(source.slice(lastIndex, match.index)));
      }

      const mark = document.createElement("mark");
      mark.className = "section-search-match";
      mark.textContent = match[0];
      fragment.appendChild(mark);
      lastIndex = match.index + match[0].length;
    }

    if (lastIndex < source.length) {
      fragment.appendChild(document.createTextNode(source.slice(lastIndex)));
    }

    return fragment;
  }

  function normalizeText(text) {
    return collapseWhitespace(text).toLowerCase();
  }

  function collapseWhitespace(text) {
    return (text || "").replace(/\s+/g, " ").trim();
  }

  function escapeRegExp(text) {
    return text.replace(/[.*+?^${}()|[\]\\]/g, "\\$&");
  }
})();

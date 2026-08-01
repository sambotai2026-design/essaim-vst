# L'ESSAIM — looper ×8 pour Launch Control XL (VST3 + Standalone)

Portage natif du prototype HTML, **fonctionnellement identique** : 8 pistes, grille auto,
mesures par piste (1/2/4/8/16/32/MAN), AUTOREC avec pré-roll, potards assignables,
sélection de piste d'entrée, monitoring, delay/reverb calés sur la grille, limiteur,
LED et soft-takeover sur le Launch Control XL.

**En plus du proto : la config audio native.** Le **standalone Windows** donne accès au
driver **ASIO** — chaque paire d'entrées/sorties de ta carte (DDJ-FLX10 comprise),
taille de buffer, fréquence. En **VST3**, l'audio est routé par le DAW.

## Compiler (sans rien installer) : GitHub Actions

1. Crée un repo GitHub et pousse ce dossier tel quel :
   ```
   git init && git add . && git commit -m "L'ESSAIM v1"
   git branch -M main
   git remote add origin https://github.com/TON_COMPTE/essaim-vst.git
   git push -u origin main
   ```
2. Onglet **Actions** → le workflow `build` démarre tout seul (~10-15 min).
3. Clique sur le run → **Artifacts** → télécharge `ESSAIM-windows` (et `ESSAIM-macos`).

## Installer

- **VST3** : copie `LESSAIM.vst3` dans `C:\Program Files\Common Files\VST3`
  (macOS : `~/Library/Audio/Plug-Ins/VST3`). Rescanne dans ton DAW.
- **Standalone** : lance `LESSAIM.exe` où tu veux. Bouton **AUDIO** → **OUVRIR LE
  PANNEAU AUDIO (ASIO)** → choisis le driver ASIO du FLX10, les canaux, le buffer.

## Sessions

Panneau **AUDIO** → section **SESSION** : `SAUVER…` / `CHARGER…` produit un fichier
`.essaim` qui embarque **tout** — boucles audio, effets, assignations, mesures,
réglages, grille. Au chargement, les pistes reviennent en **STOP, en phase** :
appuie PLAY pour les lancer ensemble. En VST3, le projet du DAW embarque
automatiquement la session complète (sauvegarder le set Ableton suffit).

## Launch Control XL

Détection automatique (entrée **et** sortie, pour les LED). Si ton DAW l'utilise déjà
comme surface de contrôle, désactive-le côté DAW : le plugin ouvre le périphérique en
direct. Sélection manuelle possible dans le panneau AUDIO, section MIDI.

Mapping identique au proto : 24 potards assignables (A/B/C), faders = volume,
boutons haut = REC/PLAY/STOP, boutons bas = CLEAR (piste sélectionnée en ambre),
flèches ◀▶ = piste d'entrée, ▲▼ = mesures de la piste sélectionnée.

## Notes techniques

- JUCE **8.0.15** épinglé (FetchContent) — ne change pas le tag sans re-tester la WebView.
- L'UI est le HTML du proto embarqué dans une WebView (WebView2 sous Windows),
  pont JSON bidirectionnel avec le moteur C++.
- Le SDK ASIO est téléchargé par la CI chez Steinberg au moment du build
  (licence Steinberg : le SDK n'est pas redistribué dans ce repo). Si le
  téléchargement échoue, le build sort quand même — sans ASIO (WASAPI/DirectSound).
- `pluginval` (niveau 5) tourne sur chaque build.
- AUTOREC : garde anti-repisse — quand des boucles jouent, le déclencheur exige un
  son qui DOMINE la sortie du looper (voix près du micro : oui ; boucle qui
  revient par le câblage ou par l'air : non). Si l'écoute est bloquée par la
  repisse, un toast le signale. Câblage requis : l'entrée du looper ne doit
  contenir QUE le micro (préampli → CH3) — jamais une boucle booth/master → CH3
  ni une paire d'entrée MIX/REC.
- AUTOREC : gate de réarmement — après chaque boucle, la piste en ÉCOUTE exige
  ~120 ms de silence sous le seuil avant d'accepter un nouveau son (la queue de
  la phrase précédente ne déclenche plus la piste suivante). Si les enceintes
  repissent dans le micro au-dessus du seuil, la piste reste en ÉCOUTE sans se
  déclencher : monte le seuil (%) ou passe au casque.
- Fin d'enregistrement échantillon-précise (le relais REC→PLAY se fait dans le
  thread audio) + fondu enchaîné ~5 ms au point de bouclage : pas de clic.

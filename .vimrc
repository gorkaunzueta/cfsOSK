set tabstop=4
set softtabstop=4
set shiftwidth=4
set noexpandtab

"""set colorcolumn=110
"""highlight ColorColumn ctermbg=darkgrey

augroup project
    autocmd!
    autocmd BufRead,BufNewFile *.h,*.c set filetype=c.doxygen
augroup END

set number
set tags=.tags

let &path.="./"

